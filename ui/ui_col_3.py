from datetime import datetime

from PyQt5.QtWidgets import QWidget, QSizePolicy
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

from .theme import Colors, Styles, Icons, Layout, QT_DIR
from .ui_test_item import TestItem


class ColTests(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_tests.ui", self)

        self._sensor_data: dict | None = None
        self._sensor_id: str | None = None
        self._runner = None
        self._repo   = None
        self._sensor_backend = None
        self._test_widgets: dict[str, TestItem] = {}
        self._runner_log_forward = None  # set by Main_UI to route logs to ColCapture

        self._setup_styles()
        self._connect_signals()

    # ── Public ────────────────────────────────────────────────────────────────

    def set_runner(self, runner, repo):
        self._runner = runner
        self._repo   = repo
        runner.log_line.connect(self._on_log_line)
        runner.test_started.connect(self._on_test_started)
        runner.test_finished.connect(self._on_test_finished)
        runner.all_finished.connect(self._on_all_finished)
        runner.error.connect(self._on_test_error)
        # Forward log lines to ColCapture if already wired by Main_UI
        if hasattr(self, '_runner_log_forward') and self._runner_log_forward:
            runner.log_line.connect(self._runner_log_forward)

    def load_sensor(self, sensor_data: dict):
        self._sensor_data    = sensor_data
        self._sensor_id      = sensor_data["id"]
        self._sensor_backend = self._make_backend(sensor_data)
        self._load_test_image(sensor_data.get("test_image_path", ""))
        self._populate_tests(sensor_data.get("tests", []))

    def append_log(self, text: str):
        # Forwarded to ColCapture via the parent MainWindow
        # also shown in col_capture; col_tests just surfaces status
        pass

    # ── Runner slots ──────────────────────────────────────────────────────────

    def _on_log_line(self, text: str):
        pass  # ColCapture handles logs

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
                pass  # logged in ColCapture

    def _on_all_finished(self):
        self.btn_run_all.setEnabled(True)

    def _on_test_error(self, test_name: str, message: str):
        w = self._test_widgets.get(test_name)
        if w:
            w.is_running  = False
            w.test_status = "Failed"
            w.test_result = f"Error: {message}"
            w.refresh()

    # ── TestItem slots ────────────────────────────────────────────────────────

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

    # ── UI button slots ───────────────────────────────────────────────────────

    def _on_run_all(self):
        if not self._can_run():
            return
        for w in self._test_widgets.values():
            w.set_running(True)
        self.btn_run_all.setEnabled(False)
        self._runner.run_tests(self._sensor_backend)

    def _on_test_description_update(self):
        # called when a test item is clicked to show its description
        pass

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _can_run(self) -> bool:
        return self._runner is not None and self._sensor_backend is not None

    def _make_backend(self, sensor_data: dict):
        try:
            from src.sensors import REGISTRY
            from config import CONFIG
            SensorType = REGISTRY.get((sensor_data["type"], sensor_data["name"]))
            if SensorType is None:
                return None
            sdf_path = sensor_data.get("sdf_path") or None
            try:
                return SensorType(CONFIG, sensor_sdf_path=sdf_path)
            except TypeError:
                return SensorType(CONFIG)
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
            w.run_requested.connect(
                lambda name: self.lbl_test_description.setText(
                    self._test_widgets[name].test_description
                    if name in self._test_widgets else ""
                )
            )
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

    # ── Styles ────────────────────────────────────────────────────────────────

    def _connect_signals(self):
        self.btn_run_all.clicked.connect(self._on_run_all)

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QWidget {{ background-color: {Colors.BG_COLUMN}; }}
            QWidget#wt_toolbar {{
                background-color: {Colors.BG_TOOLBAR};
                border-bottom: 1px solid {Colors.BORDER};
            }}
            QLabel#lbl_test_image {{
                background-color: {Colors.BG_IMAGE};
                color: {Colors.TEXT_MUTED};
                font-size: 12px;
            }}
            QLabel#lbl_test_description {{
                color: {Colors.TEXT_SECONDARY};
                font-size: 12px;
                padding: 4px 10px;
                border-bottom: 1px solid {Colors.BORDER};
                background-color: {Colors.BG_TOOLBAR};
            }}
            QScrollArea {{ border: none; background-color: transparent; }}
            QWidget#scroll_tests_contents {{ background-color: transparent; }}
            {Styles.SCROLLBAR}
        """)
        for btn, icon in [
            (self.btn_run_all,    Icons.RUN_ALL()),
            (self.btn_add_test,   Icons.ADD()),
            (self.btn_edit_tests, Icons.EDIT()),
        ]:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(Styles.BUTTON_ICON)
