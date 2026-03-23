from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

from ._theme import Colors, Styles, Icons, Layout, QT_DIR
from .widget_test_item import TestItem
from .logic_queue_manager import TestStatus


class ColTests(QWidget):

    # Emitted whenever a result arrives — connected to ColCapture.load_test_result()
    test_result_ready = pyqtSignal(str, dict)  # func_name, result

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_3.ui", self)

        self._sensor_data: dict | None = None
        self._sensor_id: str | None = None
        self._backend = None
        self._qm = None
        self._repo = None
        self._simulator = None
        self._widgets: dict[str, TestItem] = {}
        self._selected: str | None = None
        self._runner_log_forward = None

        self._test_results: dict = {}

        self._setup_heights()
        self._setup_styles()
        self._connect_toolbar()

    # ─────────────────────────────────────────
    # PUBLIC API
    # ─────────────────────────────────────────

    def set_runner(self, queue_manager, repo):
        self._qm = queue_manager
        self._repo = repo
        queue_manager.item_state_changed.connect(self._on_state_changed)
        queue_manager.item_progress_changed.connect(self._on_progress_changed)
        queue_manager.item_result.connect(self._on_item_result)

    def set_simulator(self, simulator):
        self._simulator = simulator

    def load_sensor(self, sensor_data):
        from .logic_sensor_repository import SensorRepository

        fresh = SensorRepository.instance().get_sensor(sensor_data["id"]) or sensor_data

        self._sensor_data = fresh
        self._sensor_id = str(fresh["id"])
        self._backend = self._make_backend(fresh)
        self._selected = None
        self._test_results.clear()

        self._reset_detail_panel()
        self._populate_tests(fresh.get("tests", []))
        self._restore_queue_state()

    def _restore_queue_state(self):
        """Re-apply running/queued status to widgets after a sensor reload."""
        if not self._qm or not self._sensor_id:
            return
        running = self._qm.get_running()
        if running:
            r_sid, r_func = running
            if r_sid == self._sensor_id:
                w = self._widgets.get(r_func)
                if w:
                    w.set_status(TestStatus.RUNNING)
        for func_name, status in self._qm.get_queued_for_sensor(
            self._sensor_id
        ).items():
            w = self._widgets.get(func_name)
            if w:
                w.set_status(status)

    def clear(self):
        self._sensor_data = None
        self._sensor_id = None
        self._backend = None
        self._selected = None
        self._test_results.clear()

        self._reset_detail_panel()
        self._populate_tests([])

    # ─────────────────────────────────────────
    # QUEUE SIGNALS
    # ─────────────────────────────────────────

    def _on_state_changed(self, sensor_id, func_name, status):
        if sensor_id != self._sensor_id:
            return

        w = self._widgets.get(func_name)
        if w:
            w.set_status(status)

    def _on_progress_changed(self, sensor_id, func_name, value):
        if sensor_id != self._sensor_id:
            return

        w = self._widgets.get(func_name)
        if w:
            w.set_progress(value)

    def _on_item_result(self, sensor_id, func_name, result):
        if sensor_id != self._sensor_id:
            return

        self._test_results[func_name] = result

        w = self._widgets.get(func_name)
        if w:
            w.set_result(result)

        # Forward to col_4 — always show the latest result as it arrives
        self.test_result_ready.emit(func_name, result)

    # ─────────────────────────────────────────
    # TEST ITEM ACTIONS
    # ─────────────────────────────────────────

    def _on_item_run(self, func_name):
        if not self._qm or not self._backend:
            return

        tests = self._qm.get_tests(self._backend)
        func = tests.get(func_name)

        if func:
            self._qm.enqueue(
                self._sensor_id, func_name, self._backend, func, sensor=self._backend
            )

    def _on_item_stop(self, func_name):
        if self._qm:
            self._qm.cancel(self._sensor_id, func_name)

    def _on_item_selected(self, func_name):
        if self._selected and self._selected in self._widgets:
            self._widgets[self._selected].set_selected(False)

        self._selected = func_name

        w = self._widgets.get(func_name)
        if not w:
            return

        w.set_selected(True)
        self.lbl_selected_test_name.setText(w.test_name)
        self.lbl_test_description.setText(w.test_description)
        self._load_test_image(w._image_path)

        # Re-surface cached result for this test in col_4 when re-selected
        if func_name in self._test_results:
            self.test_result_ready.emit(func_name, self._test_results[func_name])

    # ─────────────────────────────────────────
    # HELPERS
    # ─────────────────────────────────────────

    def _make_backend(self, sensor_data):
        try:
            from src.sensor import Sensor as SensorModel

            return SensorModel(
                sensor_type=sensor_data.get("type", ""),
                sensor_name=sensor_data.get("name", ""),
                sdf_path=sensor_data.get("sdf_path", ""),
                topics=sensor_data.get("topics", []),
                description=sensor_data.get("description", ""),
                image_path=sensor_data.get("image_path", ""),
                params=sensor_data.get("params", {}),
            )
        except Exception as exc:
            print(exc)
            return None

    def _populate_tests(self, tests):
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

    def _reset_detail_panel(self):
        self.lbl_test_image.clear()
        self.lbl_test_image.setText("no test image")
        self.lbl_selected_test_name.setText("")
        self.lbl_test_description.setText("")

    def _load_test_image(self, path):
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

    # ─────────────────────────────────────────
    # UI SETUP
    # ─────────────────────────────────────────

    def _setup_heights(self):
        self.lbl_test_image.setFixedHeight(Layout.IMAGE_H)
        self.wt_toolbar.setFixedHeight(Layout.TOOLBAR_H)
        self.lbl_selected_test_name.setFixedHeight(Layout.NAME_H)
        self.scroll_description.setFixedHeight(Layout.DESC_H)

    def _connect_toolbar(self):
        self.btn_run_all.clicked.connect(self._on_run_all)
        self.btn_edit_tests.clicked.connect(self._on_edit_tests)

    def _on_tests_saved(self):
        """Reload the current sensor so updated test meta is reflected."""
        if not self._sensor_data:
            return
        from .logic_sensor_repository import SensorRepository

        fresh = SensorRepository.instance().get_sensor(self._sensor_data["id"])
        if fresh:
            self.load_sensor(fresh)

    def _on_run_all(self):
        if not self._qm or not self._backend:
            return

        tests = self._qm.get_tests(self._backend)

        for func_name, func in tests.items():
            self._qm.enqueue(
                self._sensor_id, func_name, self._backend, func, sensor=self._backend
            )

    def _on_edit_tests(self):
        if not self._sensor_data:
            return

        from .dialog_edit_tests import EditTestsDialog
        from .logic_sensor_repository import SensorRepository

        sensor_id = self._sensor_id
        sensor_name = self._sensor_data.get("name", "")

        # Build test list: start from what the queue manager knows,
        # then overlay display meta from the repo.
        tests = []
        if self._qm and self._backend:
            repo = SensorRepository.instance()
            meta_map = repo.get_test_meta(
                sensor_id
            )  # {func_name: {display_name, description, image_path}}
            core_tests = self._qm.get_tests(self._backend)
            core_names = set(core_tests.keys())

            # Tests present in core
            for func_name in core_tests:
                meta = meta_map.get(func_name, {})
                tests.append(
                    {
                        "func_name": func_name,
                        "display_name": meta.get("display_name") or func_name,
                        "description": meta.get("description") or "",
                        "image_path": meta.get("image_path") or "",
                        "missing": False,
                    }
                )

            # Tests in meta but not in core (stale / renamed)
            for func_name, meta in meta_map.items():
                if func_name not in core_names:
                    tests.append(
                        {
                            "func_name": func_name,
                            "display_name": meta.get("display_name") or func_name,
                            "description": meta.get("description") or "",
                            "image_path": meta.get("image_path") or "",
                            "missing": True,
                        }
                    )
        else:
            # Fallback: use whatever is stored in sensor_data tests
            for t in self._sensor_data.get("tests", []):
                tests.append(
                    {
                        "func_name": t.get("name", ""),
                        "display_name": t.get("display_name") or t.get("name", ""),
                        "description": t.get("description") or "",
                        "image_path": t.get("image_path") or "",
                        "missing": False,
                    }
                )

        dlg = EditTestsDialog(
            sensor_id=sensor_id,
            sensor_name=sensor_name,
            tests=tests,
            parent=self,
        )
        dlg.tests_saved.connect(self._on_tests_saved)
        dlg.exec_()

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QWidget#ColTests {{ background-color: {Colors.BG_COLUMN}; }}
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
            QScrollArea#scroll_description,
            QScrollArea#scroll_tests {{
                border: none;
                background: transparent;
            }}
            QWidget#scroll_description_contents,
            QWidget#scroll_tests_contents {{
                background: transparent;
            }}
            {Styles.SCROLLBAR}
        """)

        self.scroll_description.setStyleSheet(Styles.DESCRIPTION_AREA)

        for btn, icon in [
            (self.btn_run_all, Icons.RUN_ALL()),
            (self.btn_edit_tests, Icons.EDIT()),
        ]:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(Styles.BUTTON_ICON)
