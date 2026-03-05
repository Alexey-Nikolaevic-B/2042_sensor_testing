import os

from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import QSize, pyqtSignal
from PyQt5.QtGui import QIcon
from PyQt5 import uic

from theme import Styles, Icons, Layout, QT_DIR


class TestItem(QWidget):

    run_requested  = pyqtSignal(str)
    stop_requested = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.is_running       = False
        self.test_name        = ""
        self.test_status      = "Pending"
        self.test_description = ""
        self.test_result      = ""

        self._init_ui()
        self._setup_styles()
        self._connect_signals()

    def _init_ui(self):
        ui_path = os.path.join(os.path.dirname(__file__), QT_DIR, "test_item.ui")
        uic.loadUi(ui_path, self)

    def _setup_styles(self):
        self.setStyleSheet("QWidget#Test_Item > QWidget { background-color: transparent; }")
        self.btn_run_stop.setStyleSheet(Styles.BUTTON_ICON)
        self.progress_bar.setStyleSheet(Styles.PROGRESS_BAR)
        self.lbl_test_name.setStyleSheet("""
            QLabel {
                color: rgb(200, 200, 200);
                background-color: transparent;
                font-weight: bold;
                font-size: 14px;
            }
        """)
        self.test_result_label.setStyleSheet("""
            QLabel {
                color: rgb(150, 150, 150);
                background-color: transparent;
                font-size: 12px;
            }
        """)
        self.status_icon.setStyleSheet("background-color: transparent;")

    def _connect_signals(self):
        self.btn_run_stop.clicked.connect(self._on_run_stop_clicked)

    def load(self, test_data: dict):
        self.test_name        = test_data.get("name", "")
        self.test_status      = test_data.get("status", "Pending")
        self.test_description = test_data.get("description", "")
        self.test_result      = str(test_data.get("result", ""))
        self.is_running       = False
        self.refresh()

    def refresh(self):
        self.lbl_test_name.setText(self.test_name)
        self._refresh_run_stop_btn()
        self._refresh_progress_bar()
        self._refresh_status_icon()

    def set_running(self, running: bool):
        self.is_running = running
        self.refresh()

    def _on_run_stop_clicked(self):
        if self.is_running:
            self.stop_requested.emit(self.test_name)
        else:
            self.run_requested.emit(self.test_name)

    def _refresh_run_stop_btn(self):
        icon = Icons.STOP() if self.is_running else Icons.RUN()
        self.btn_run_stop.setIcon(icon)
        self.btn_run_stop.setIconSize(Layout.ICON_SIZE_MD)

    def _refresh_progress_bar(self):
        if self.is_running:
            self.progress_bar.show()
            self.test_result_label.hide()
        else:
            self.progress_bar.hide()
            self.test_result_label.show()

    def _refresh_status_icon(self):
        self.test_result_label.setText(self.test_result)
        self.status_icon.setPixmap(
            Icons.for_status(self.test_status, self.is_running).pixmap(Layout.ICON_SIZE_MD)
        )