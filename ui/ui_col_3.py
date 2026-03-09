from datetime import datetime

from PyQt5.QtWidgets import QWidget, QSizePolicy
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

from .theme import Colors, Styles, Icons, Layout, QT_DIR
from .ui_test_item import TestItem
from .ui_col_2 import DESCRIPTION_STYLE, IMAGE_H, TOOLBAR_H, NAME_H, DESC_H


class ColTests(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_3.ui", self)

        self._sensor_data: dict | None = None
        self._sensor_id: str | None = None
        self._runner = None
        self._repo   = None
        self._sensor_backend = None
        self._test_widgets: dict[str, TestItem] = {}
        self._selected_test: str | None = None
        self._runner_log_forward = None

        self._enforce_heights()
        self._setup_styles()
        self._connect_signals()

    def set_runner(self, runner, repo):
        self._runner = runner
        self._repo   = repo
        runner.log_line.connect(self._on_log_line)
        runner.test_started.connect(self._on_test_started)
        runner.test_finished.connect(self._on_test_finished)
        runner.test_progress.connect(self._on_test_progress)
        runner.all_finished.connect(self._on_all_finished)
        runner.error.connect(self._on_test_error)
        if self._runner_log_forward:
            runner.log_line.connect(
                lambda text: self._runner_log_forward("info", "test_runner", text)
            )

    def load_sensor(self, sensor_data: dict):
        from .sensor_repository import SensorRepository
        fresh = SensorRepository.instance().get_sensor(sensor_data["id"]) or sensor_data
        self._sensor_data    = fresh
        self._sensor_id      = fresh["id"]
        self._sensor_backend = self._make_backend(fresh)
        self._selected_test  = None
        self.lbl_test_image.clear()
        self.lbl_test_image.setText("no test image")
        self.lbl_selected_test_name.setText("")
        self.lbl_test_description.setText("")
        self._populate_tests(fresh.get("tests", []))

    def clear(self):
        self._sensor_data    = None
        self._sensor_id      = None
        self._sensor_backend = None
        self.lbl_test_image.clear()
        self.lbl_test_image.setText("no test image")
        self.lbl_selected_test_name.setText("—")
        self.lbl_test_description.setText("")
        self._populate_tests([])

    def append_log(self, text: str):
        pass

    def _on_log_line(self, text: str):
        pass

    def _on_test_started(self, test_name: str):
        w = self._test_widgets.get(test_name)
        if w:
            w.set_running(True)

    def _on_test_finished(self, test_name: str, result: dict, status: str, duration: float):
        w = self._test_widgets.get(test_name)
        if w:
            w.is_running  = False
            w.test_status = status
            w.test_result = self._fmt_result(result)
            w.refresh()
        if self._repo and self._sensor_id:
            try:
                self._repo.save_test_result(
                    sensor_id=self._sensor_id,
                    test_name=test_name,
                    status=status,
                    result=result,
                    duration=duration,
                )
            except Exception as exc:
                pass

    def _on_all_finished(self):
        self.btn_run_all.setEnabled(True)

    def _on_test_error(self, test_name: str, message: str):
        w = self._test_widgets.get(test_name)
        if w:
            w.is_running  = False
            w.test_status = "Failed"
            w.test_result = f"Error: {message}"
            w.refresh()

    def _on_test_progress(self, test_name: str, value: int):
        w = self._test_widgets.get(test_name)
        if w:
            w.set_progress(value)

    def _on_item_selected(self, func_name: str):
        if self._selected_test and self._selected_test in self._test_widgets:
            self._test_widgets[self._selected_test].set_selected(False)
        self._selected_test = func_name
        w = self._test_widgets.get(func_name)
        if w:
            w.set_selected(True)
            self.lbl_selected_test_name.setText(w.test_name)
            self.lbl_test_description.setText(w.test_description)
            self._load_test_image(w._image_path)

    def _on_item_run(self, test_name: str):
        if not self._can_run():
            return
        w = self._test_widgets.get(test_name)
        if w:
            w.set_running(True)
        self._runner.run_tests(self._sensor_backend, test_names=[test_name])

    def _on_item_stop(self, test_name: str):
        self._runner.stop()
        w = self._test_widgets.get(test_name)
        if w:
            w.set_running(False)

    def _on_run_all(self):
        if not self._can_run():
            return
        for w in self._test_widgets.values():
            w.set_running(True)
        self.btn_run_all.setEnabled(False)
        self._runner.run_tests(self._sensor_backend)

    def _on_test_description_update(self):
        pass

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

        sensor_id   = self._sensor_id
        sensor_name = self._sensor_data.get("name", "")
        repo        = SensorRepository.instance()

        live_funcs = {}
        if self._runner and hasattr(self._runner, "_core") and self._sensor_backend is not None:
            try:
                live_funcs = self._runner._core.get_tests(self._sensor_backend)
            except Exception as exc:
                print(f"[EditTests] get_tests failed: {exc}")

        saved_meta = repo.get_test_meta(sensor_id)

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
        if self._selected_test and self._selected_test in self._test_widgets:
            w = self._test_widgets[self._selected_test]
            w.set_selected(True)
            self.lbl_selected_test_name.setText(w.test_name)
            self.lbl_test_description.setText(w.test_description)
            self._load_test_image(w._image_path)

    def _can_run(self) -> bool:
        return self._runner is not None and self._sensor_backend is not None

    def _make_backend(self, sensor_data: dict):
        try:
            from src.sensors import REGISTRY
            from config import CONFIG
            SensorClass = next(
                (cls for (stype, _), cls in REGISTRY.items() if stype == sensor_data["type"]),
                None,
            )
            if SensorClass is None:
                return None
            sdf_path = sensor_data.get("sdf_path") or None
            try:
                return SensorClass(CONFIG, sensor_sdf_path=sdf_path)
            except TypeError:
                return SensorClass(CONFIG)
        except Exception:
            return None

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

    def _populate_tests(self, tests: list[dict]):
        layout = self.scroll_tests_contents.layout()
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._test_widgets.clear()

        for test_data in tests:
            w = TestItem(parent=self)
            w.load(test_data)
            w.run_requested.connect(self._on_item_run)
            w.stop_requested.connect(self._on_item_stop)
            w.selected.connect(self._on_item_selected)
            layout.addWidget(w)
            self._test_widgets[test_data["name"]] = w

        layout.addStretch(1)

    @staticmethod
    def _fmt_result(result) -> str:
        if not result:
            return ""
        if isinstance(result, dict):
            return " | ".join(
                f"{k}: {v}" for k, v in result.items() if k != "passed"
            )
        return str(result)

    def _enforce_heights(self):
        self.lbl_test_image.setFixedHeight(IMAGE_H)
        self.wt_toolbar.setFixedHeight(TOOLBAR_H)
        self.lbl_selected_test_name.setFixedHeight(NAME_H)
        self.scroll_description.setFixedHeight(DESC_H)

    def _connect_signals(self):
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