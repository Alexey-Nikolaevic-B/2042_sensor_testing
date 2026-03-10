from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

from .theme import Colors, Styles, Icons, Layout, QT_DIR
from .ui_test_item import TestItem
from .ui_col_2 import DESCRIPTION_STYLE, IMAGE_H, TOOLBAR_H, NAME_H, DESC_H
from .queue_manager import TestStatus


class ColTests(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_3.ui", self)

        self._sensor_data: dict | None = None
        self._sensor_id:   str  | None = None
        self._backend      = None
        self._qm           = None
        self._repo         = None
        self._widgets: dict[str, TestItem] = {}
        self._selected: str | None = None
        self._runner_log_forward = None   # set by ui_main: col_4.append_log

        self._enforce_heights()
        self._setup_styles()
        self._connect_toolbar()


    def set_runner(self, queue_manager, repo):
        self._qm   = queue_manager
        self._repo = repo
        queue_manager.item_state_changed.connect(self._on_state_changed)
        queue_manager.item_progress_changed.connect(self._on_progress_changed)
        queue_manager.item_result.connect(self._on_item_result)
        if self._runner_log_forward:
            queue_manager.log_line.connect(
                lambda text: self._runner_log_forward("info", "test_runner", text)
            )

    def _log(self, level: str, msg: str):
        if self._runner_log_forward:
            self._runner_log_forward(level, "col_3", msg)
        else:
            print(f"[col_3/{level}] {msg}")

    def load_sensor(self, sensor_data: dict):
        from .sensor_repository import SensorRepository
        fresh = SensorRepository.instance().get_sensor(sensor_data["id"]) or sensor_data
        self._sensor_data = fresh
        self._sensor_id   = str(fresh["id"])
        self._backend     = self._make_backend(fresh)   # may be None; errors logged
        self._selected    = None
        self._reset_detail_panel()
        self._populate_tests(fresh.get("tests", []))

    def clear(self):
        self._sensor_data = None
        self._sensor_id   = None
        self._backend     = None
        self._selected    = None
        self._reset_detail_panel()
        self._populate_tests([])

    def _on_state_changed(self, sensor_id: str, func_name: str, status: TestStatus):
        try:
            if sensor_id != self._sensor_id:
                return
            w = self._widgets.get(func_name)
            if w:
                w.set_status(status)
            self._sync_run_all_button()
        except Exception as exc:
            self._log("error", f"_on_state_changed error: {exc}")

    def _on_progress_changed(self, sensor_id: str, func_name: str, value: int):
        try:
            if sensor_id != self._sensor_id:
                return
            w = self._widgets.get(func_name)
            if w:
                w.set_progress(value)
        except Exception as exc:
            self._log("error", f"_on_progress_changed error: {exc}")

    def _on_item_result(self, sensor_id: str, func_name: str, result: dict):
        try:
            if sensor_id != self._sensor_id:
                return
            w = self._widgets.get(func_name)
            if w:
                w.set_result(result)
        except Exception as exc:
            self._log("error", f"_on_item_result error: {exc}")


    def _on_item_run(self, func_name: str):
        if not self._qm:
            return

        if not self._backend:
            self._log("error", f"Cannot run '{func_name}': sensor backend failed to load. Check sensor type/SDF.")
            return

        tests = self._qm.get_tests(self._backend)
        func  = tests.get(func_name)

        if func is None:
            self._log("error", f"Cannot run '{func_name}': test function not found. Available: {list(tests.keys())}")
            return

        self._qm.enqueue(self._sensor_id, func_name, self._backend, func)

    def _on_item_stop(self, func_name: str):
        if self._qm:
            self._qm.cancel(self._sensor_id, func_name)

    def _on_run_all(self):
        if not self._qm:
            return

        if not self._backend:
            self._log("error", "Cannot run tests: sensor backend failed to load. Check sensor type/SDF.")
            return

        tests = self._qm.get_tests(self._backend)
        queued = 0
        skip_statuses = (TestStatus.QUEUED, TestStatus.RUNNING)
        for func_name, func in tests.items():
            w = self._widgets.get(func_name)
            if w and w.test_status not in skip_statuses:
                self._qm.enqueue(self._sensor_id, func_name, self._backend, func)
                queued += 1

        if queued == 0:
            self._log("info", "All tests are already queued or running.")

    def _on_item_selected(self, func_name: str):
        if self._selected and self._selected in self._widgets:
            self._widgets[self._selected].set_selected(False)
        self._selected = func_name
        w = self._widgets.get(func_name)
        if w:
            w.set_selected(True)
            self.lbl_selected_test_name.setText(w.test_name)
            self.lbl_test_description.setText(w.test_description)
            self._load_test_image(w._image_path)


    def _on_edit_tests(self):
        if not self._sensor_data:
            return
        try:
            self._open_edit_tests_dialog()
        except Exception:
            import traceback
            traceback.print_exc()

    def _open_edit_tests_dialog(self):
        from .ui_edit_tests_dialog import EditTestsDialog
        from .sensor_repository import SensorRepository

        repo        = SensorRepository.instance()
        sensor_id   = self._sensor_id
        sensor_name = self._sensor_data.get("name", "")
        saved_meta  = repo.get_test_meta(sensor_id)

        live_funcs = {}
        if self._qm and self._backend:
            try:
                live_funcs = self._qm.get_tests(self._backend)
            except Exception as exc:
                self._log("error", f"get_tests failed: {exc}")

        tests = []
        for func_name in live_funcs:
            meta = saved_meta.get(func_name, {})
            tests.append({
                "func_name":    func_name,
                "display_name": meta.get("display_name") or func_name,
                "description":  meta.get("description", ""),
                "image_path":   meta.get("image_path", ""),
                "missing":      False,
            })
        for func_name, meta in saved_meta.items():
            if func_name not in live_funcs:
                tests.append({
                    "func_name":    func_name,
                    "display_name": meta.get("display_name") or func_name,
                    "description":  meta.get("description", ""),
                    "image_path":   meta.get("image_path", ""),
                    "missing":      True,
                })

        dlg = EditTestsDialog(sensor_id, sensor_name, tests, parent=self)
        dlg.tests_saved.connect(self._on_tests_meta_saved)
        dlg.exec_()

    def _on_tests_meta_saved(self):
        if not self._sensor_id:
            return
        from .sensor_repository import SensorRepository
        sensor = SensorRepository.instance().get_sensor(self._sensor_id)
        if not sensor:
            return
        self._sensor_data = sensor
        self._populate_tests(sensor.get("tests", []))
        if self._selected and self._selected in self._widgets:
            w = self._widgets[self._selected]
            w.set_selected(True)
            self.lbl_selected_test_name.setText(w.test_name)
            self.lbl_test_description.setText(w.test_description)
            self._load_test_image(w._image_path)


    def _sync_run_all_button(self):
        pass  # btn_run_all is always enabled — clicking re-queues any idle tests

    def _make_backend(self, sensor_data: dict):
        try:
            from src.sensors import REGISTRY
            from config import CONFIG

            sensor_type = sensor_data.get("type")
            SensorClass = next(
                (cls for (stype, _), cls in REGISTRY.items() if stype == sensor_type),
                None,
            )
            if SensorClass is None:
                self._log("error",
                    f"No sensor class registered for type={sensor_type!r}. "
                    f"Known types: {sorted({k[0] for k in REGISTRY})}"
                )
                return None

            sdf = sensor_data.get("sdf_path") or None
            try:
                return SensorClass(CONFIG, sensor_sdf_path=sdf)
            except TypeError:
                return SensorClass(CONFIG)

        except Exception as exc:
            import traceback
            self._log("error", f"Backend init failed: {exc}\n{traceback.format_exc()}")
            return None

    def _populate_tests(self, tests: list):
        layout = self.scroll_tests_contents.layout()
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._widgets.clear()

        for td in tests:
            w = TestItem(parent=self)
            w.load(td)
            w.run_requested.connect(self._on_item_run)
            w.stop_requested.connect(self._on_item_stop)
            w.selected.connect(self._on_item_selected)
            layout.addWidget(w)
            self._widgets[td["name"]] = w

        layout.addStretch(1)

        if self._qm and self._sensor_id:
            self._restore_live_states()

        self._sync_run_all_button()

    def _restore_live_states(self):
        running = self._qm.get_running()
        queued  = self._qm.get_queued_for_sensor(self._sensor_id)

        for func_name, status in queued.items():
            w = self._widgets.get(func_name)
            if w:
                w.set_status(status)

        if running and running[0] == self._sensor_id:
            w = self._widgets.get(running[1])
            if w:
                w.set_status(TestStatus.RUNNING)

    def _reset_detail_panel(self):
        self.lbl_test_image.clear()
        self.lbl_test_image.setText("no test image")
        self.lbl_selected_test_name.setText("")
        self.lbl_test_description.setText("")

    def _load_test_image(self, path: str):
        if path:
            px = QPixmap(path)
            if not px.isNull():
                self.lbl_test_image.setPixmap(
                    px.scaled(
                        self.lbl_test_image.width(),
                        self.lbl_test_image.height(),
                        Qt.KeepAspectRatio,
                        Qt.SmoothTransformation,
                    )
                )
                return
        self.lbl_test_image.clear()
        self.lbl_test_image.setText("no test image")

    def _enforce_heights(self):
        self.lbl_test_image.setFixedHeight(IMAGE_H)
        self.wt_toolbar.setFixedHeight(TOOLBAR_H)
        self.lbl_selected_test_name.setFixedHeight(NAME_H)
        self.scroll_description.setFixedHeight(DESC_H)

    def _connect_toolbar(self):
        self.btn_run_all.clicked.connect(self._on_run_all)
        self.btn_edit_tests.clicked.connect(self._on_edit_tests)

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QWidget#ColSensors {{ background-color: {Colors.BG_COLUMN}; }}
            QWidget#wt_toolbar {{
                background-color: {Colors.BG_TOOLBAR};
                border-bottom: 1px solid {Colors.BORDER};
            }}
            QLabel#lbl_test_image {{
                background-color: {Colors.BG_IMAGE};
                color: {Colors.TEXT_MUTED};
                font-size: 12px;
            }}
            QLabel#lbl_selected_test_name {{
                color: {Colors.TEXT_WHITE};
                font-size: 15px;
                font-weight: bold;
                padding: 0 12px;
            }}
            QScrollArea {{ border: none; background: transparent; }}
            QWidget#scroll_tests_contents {{ background: transparent; }}
            {Styles.SCROLLBAR}
        """)
        self.scroll_description.setStyleSheet(DESCRIPTION_STYLE)
        for btn, icon in [
            (self.btn_run_all,    Icons.RUN_ALL()),
            (self.btn_add_test,   Icons.ADD()),
            (self.btn_edit_tests, Icons.EDIT()),
        ]:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(Styles.BUTTON_ICON)