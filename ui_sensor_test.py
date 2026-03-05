import os
from datetime import datetime

from PyQt5.QtWidgets import QWidget, QListWidgetItem, QSizePolicy, QFileDialog, QApplication
from PyQt5.QtCore import Qt, QSize
from PyQt5 import uic

from ui_test_item import TestItem
from theme import Styles, Icons, Layout, QT_DIR


class TestingWindow(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)

        self._sensor_data    = None
        self._sensor_id      = None
        self._runner         = None
        self._repo           = None
        self._sensor_backend = None

        self._test_widgets: dict[str, TestItem] = {}

        self._init_ui()
        self._setup_styles()
        self._connect_signals()

    def _init_ui(self):
        uic.loadUi(f"{QT_DIR}/sensor_test.ui", self)

    def _setup_styles(self):
        self.setStyleSheet(f"""
            *:focus {{ outline: none; }}
            QLabel {{ color: #e0e0e0; }}
            QLabel#sensor_view_label,
            QLabel#sensor_image,
            QLabel#test_image,
            QLabel#label_log {{
                background-color: #000000;
                color: #ffffff;
                font-family: monospace;
                font-size: 12px;
            }}
            QLabel#label_test_description {{ background-color: transparent; }}
            QFrame#test_bar,
            QFrame#frame_test_description,
            QFrame#frame_sensor_view_2 {{ background-color: #252525; }}
            QGroupBox, QFrame, QWidget#container,
            QWidget#main_container, QWidget#content {{
                outline: none; border: none;
            }}
            {Styles.GROUP_BOX}
            {Styles.LIST_WIDGET}
            {Styles.SCROLLBAR_HIDDEN}
        """)

        for btn, icon in [
            (self.btn_run_all,    Icons.RUN_ALL()),
            (self.btn_clear_logs, Icons.CLEAR()),
            (self.btn_copy_logs,  Icons.COPY()),
            (self.btn_save_logs,  Icons.SAVE()),
        ]:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(Styles.BUTTON_ICON)

    def _connect_signals(self):
        self.list_tests.itemSelectionChanged.connect(self._on_test_selected)
        self.btn_run_all.clicked.connect(self._on_run_all_clicked)
        self.btn_clear_logs.clicked.connect(lambda: self.label_log.setText(""))
        self.btn_copy_logs.clicked.connect(self._on_copy_logs_clicked)
        self.btn_save_logs.clicked.connect(self._on_save_logs_clicked)

    def set_runner(self, runner, repo):
        self._runner = runner
        self._repo   = repo

        runner.log_line.connect(self.append_log)
        runner.test_started.connect(self._on_test_started)
        runner.test_finished.connect(self._on_test_finished)
        runner.all_finished.connect(self._on_all_finished)
        runner.error.connect(self._on_test_error)

    def load_sensor(self, sensor_data: dict):
        self._sensor_data    = sensor_data
        self._sensor_id      = sensor_data["id"]
        self._sensor_backend = self._make_backend(sensor_data)
        self.label_log.setText("")
        self._test_widgets.clear()
        self._populate_test_list()

    def append_log(self, text: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        new_line  = f"[{timestamp}] {text}"
        current   = self.label_log.text()
        self.label_log.setText((current + "\n" + new_line).lstrip())
        self.label_log.repaint()

    def _on_test_started(self, test_name: str):
        w = self._test_widgets.get(test_name)
        if w:
            w.set_running(True)

    def _on_test_finished(self, test_name: str, result: dict, status: str, duration: float):
        w = self._test_widgets.get(test_name)
        if w:
            w.is_running  = False
            w.test_status = status
            w.test_result = self._format_result(result)
            w.refresh()

        if self._repo and self._sensor_id:
            try:
                self._repo.save_test_result(
                    sensor_id = self._sensor_id,
                    test_name = test_name,
                    status    = status,
                    result    = result,
                    duration  = duration,
                )
            except Exception as exc:
                self.append_log(f"[DB] Could not save {test_name}: {exc}")

    def _on_all_finished(self):
        self.append_log("All tests complete.")
        self.btn_run_all.setEnabled(True)

    def _on_test_error(self, test_name: str, message: str):
        w = self._test_widgets.get(test_name)
        if w:
            w.is_running  = False
            w.test_status = "Failed"
            w.test_result = f"Error: {message}"
            w.refresh()

    def _on_item_run_requested(self, test_name: str):
        if not self._can_run():
            return
        w = self._test_widgets.get(test_name)
        if w:
            w.set_running(True)
        self._runner.run_tests(self._sensor_backend, test_names=[test_name])

    def _on_item_stop_requested(self, test_name: str):
        self._runner.stop()
        w = self._test_widgets.get(test_name)
        if w:
            w.set_running(False)

    def _on_test_selected(self):
        selected = self.list_tests.selectedItems()
        if selected:
            w = self.list_tests.itemWidget(selected[0])
            if w:
                self.label_test_description.setText(w.test_description)

    def _on_run_all_clicked(self):
        if not self._can_run():
            return
        for w in self._test_widgets.values():
            w.set_running(True)
        self.btn_run_all.setEnabled(False)
        self._runner.run_tests(self._sensor_backend)

    def _on_copy_logs_clicked(self):
        text = self.label_log.text()
        if text:
            QApplication.clipboard().setText(text)

    def _on_save_logs_clicked(self):
        text = self.label_log.text()
        if not text:
            return
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Logs",
            f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
            "Text Files (*.txt);;All Files (*)",
        )
        if path:
            try:
                with open(path, "w") as f:
                    f.write(text)
            except OSError as e:
                self.append_log(f"Error saving: {e}")

    def _can_run(self) -> bool:
        if self._runner is None:
            self.append_log("Runner not initialised.")
            return False
        if self._sensor_backend is None:
            name = self._sensor_data.get("name", "?") if self._sensor_data else "?"
            self.append_log(f"'{name}' not found in backend registry.")
            return False
        return True

    def _make_backend(self, sensor_data: dict):
        try:
            from src.sensors import REGISTRY
            from config import CONFIG

            sensor_type = sensor_data["type"]
            sensor_name = sensor_data["name"]
            sdf_path    = sensor_data.get("sdf_path", "") or None

            SensorType = REGISTRY.get((sensor_type, sensor_name))
            if SensorType is None:
                self.append_log(
                    f"'{sensor_name}' is not registered in the backend registry."
                )
                return None

            try:
                backend = SensorType(CONFIG, sensor_sdf_path=sdf_path)
            except TypeError:
                backend = SensorType(CONFIG)

            return backend

        except Exception as exc:
            self.append_log(f"Could not create backend sensor: {exc}")
            return None

    def _populate_test_list(self):
        self.list_tests.clear()
        self._test_widgets.clear()

        tests = self._sensor_data.get("tests", [])
        for test_data in tests:
            list_item = QListWidgetItem(self.list_tests)
            list_item.setSizeHint(QSize(400, Layout.TEST_ITEM_HEIGHT))

            w = TestItem(parent=self)
            w.load(test_data)
            w.run_requested.connect(self._on_item_run_requested)
            w.stop_requested.connect(self._on_item_stop_requested)

            self.list_tests.addItem(list_item)
            self.list_tests.setItemWidget(list_item, w)
            self._test_widgets[test_data["name"]] = w

        self._resize_test_list(len(tests))

    def _resize_test_list(self, count: int):
        spacing      = self.list_tests.spacing() or 0
        frame_margin = self.list_tests.frameWidth() * 2
        total        = count * (Layout.TEST_ITEM_HEIGHT + spacing) + frame_margin
        self.list_tests.setFixedHeight(min(Layout.TEST_LIST_MAX_H, total))
        self.list_tests.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.scrollArea_tests.setWidgetResizable(True)
        self.scrollArea_tests.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        layout = self.scrollAreaWidgetContents_tests.layout()
        if layout:
            layout.setAlignment(Qt.AlignTop)

    @staticmethod
    def _format_result(result) -> str:
        if not result:
            return ""
        if isinstance(result, dict):
            return " | ".join(f"{k}: {v}" for k, v in result.items() if k != "passed")
        return str(result)