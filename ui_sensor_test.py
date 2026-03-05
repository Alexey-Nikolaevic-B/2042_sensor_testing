import os
from datetime import datetime

from PyQt5.QtWidgets import QWidget, QListWidgetItem, QSizePolicy, QFileDialog, QApplication
from PyQt5.QtCore import Qt, QSize
from PyQt5 import uic

from queue_manager import QueueManager
from ui_test_item import TestItem
from theme import Styles, Icons, Layout, QT_DIR


class TestingWindow(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)

        self._queue       = QueueManager()
        self._sensor_data = None

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
                font-weight: bold;
            }}
            QLabel#label_test_description {{ background-color: transparent; }}
            QFrame#test_bar,
            QFrame#frame_test_description,
            QFrame#frame_sensor_view_2 {{ background-color: #252525; }}
            QGroupBox, QFrame, QWidget#container,
            QWidget#main_container, QWidget#content {{
                outline: none;
                border: none;
            }}
            {Styles.GROUP_BOX}
            {Styles.LIST_WIDGET}
            {Styles.SCROLLBAR_HIDDEN}
        """)

        self.btn_run_all.setIcon(Icons.RUN_ALL())
        self.btn_run_all.setIconSize(Layout.ICON_SIZE_MD)
        self.btn_run_all.setStyleSheet(Styles.BUTTON_ICON)

        self.btn_clear_logs.setStyleSheet(Styles.BUTTON_DEFAULT)

        self.btn_copy_logs.setStyleSheet(Styles.BUTTON_DEFAULT)

        self.btn_save_logs.setStyleSheet(Styles.BUTTON_DEFAULT)

    def _connect_signals(self):
        self.list_tests.itemSelectionChanged.connect(self._on_test_selected)
        self.btn_run_all.clicked.connect(self._on_run_all_clicked)
        self.btn_clear_logs.clicked.connect(self._on_clear_logs_clicked)
        self.btn_copy_logs.clicked.connect(self._on_copy_logs_clicked)
        self.btn_save_logs.clicked.connect(self._on_save_logs_clicked)

    def load_sensor(self, sensor_data: dict):
        self._sensor_data = sensor_data
        self._queue.clear()
        self.label_log.setText("")
        self._populate_test_list()

    def append_log(self, text: str):
        current = self.label_log.text()
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.label_log.setText(f"{current}[{timestamp}] {text}\n".lstrip())

    def _on_test_selected(self):
        selected = self.list_tests.selectedItems()
        if selected:
            widget = self.list_tests.itemWidget(selected[0])
            if widget:
                self.label_test_description.setText(widget.test_description)

    def _on_run_all_clicked(self):
        for i in range(self.list_tests.count()):
            item = self.list_tests.item(i)
            widget = self.list_tests.itemWidget(item)
            if widget and not widget.is_running:
                widget.is_running = True
                self._queue.add_to_queue(widget)
                widget.refresh()

    def _on_clear_logs_clicked(self):
        self.label_log.setText("")

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
                self.append_log(f"Error saving file: {e}")

    def _populate_test_list(self):
        self.list_tests.clear()

        tests = self._sensor_data.get("tests", [])
        for test_data in tests:
            list_item = QListWidgetItem(self.list_tests)
            list_item.setSizeHint(QSize(400, Layout.TEST_ITEM_HEIGHT))

            test_widget = TestItem(self._queue, parent=self)
            test_widget.load(test_data)
            test_widget.list_item = list_item

            self.list_tests.addItem(list_item)
            self.list_tests.setItemWidget(list_item, test_widget)

        self._resize_test_list(len(tests))

    def _resize_test_list(self, test_count: int):
        spacing      = self.list_tests.spacing() or 0
        frame_margin = self.list_tests.frameWidth() * 2
        total_height = test_count * (Layout.TEST_ITEM_HEIGHT + spacing) + frame_margin
        self.list_tests.setFixedHeight(min(Layout.TEST_LIST_MAX_H, total_height))
        self.list_tests.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self.scrollArea_tests.setWidgetResizable(True)
        self.scrollArea_tests.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        layout = self.scrollAreaWidgetContents_tests.layout()
        if layout:
            layout.setAlignment(Qt.AlignTop)