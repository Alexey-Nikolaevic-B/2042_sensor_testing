import os
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal
from PyQt5 import uic

from .theme import Styles, Icons, Layout, Colors, QT_DIR


class TestItem(QWidget):
    run_requested  = pyqtSignal(str)
    stop_requested = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(os.path.join(QT_DIR, "test_item.ui"), self)

        self.is_running   = False
        self.test_name    = ""
        self.test_status  = "Pending"
        self.test_description = ""
        self.test_result  = ""

        self._setup_styles()
        self.btn_run_stop.clicked.connect(self._on_run_stop_clicked)

    # ── Public ────────────────────────────────────────────────────────────────

    def load(self, test_data: dict):
        self.test_name        = test_data.get("name", "")
        self.test_status      = test_data.get("status", "Pending")
        self.test_description = test_data.get("description", "")
        self.test_result      = str(test_data.get("result", ""))
        self.is_running       = False
        self.refresh()

    def refresh(self):
        self.lbl_test_name.setText(self.test_name)
        self._refresh_status_bar()
        self._refresh_button()
        self._refresh_progress()
        self._refresh_icon()

    def set_running(self, running: bool):
        self.is_running = running
        self.refresh()

    # ── Private ───────────────────────────────────────────────────────────────

    def _on_run_stop_clicked(self):
        if self.is_running:
            self.stop_requested.emit(self.test_name)
        else:
            self.run_requested.emit(self.test_name)

    def _refresh_status_bar(self):
        color = {
            "Passed":  Colors.STATUS_GREEN,
            "Failed":  Colors.STATUS_RED,
            "Pending": Colors.STATUS_YELLOW,
        }.get(self.test_status, Colors.STATUS_BLUE)
        if self.is_running:
            color = Colors.STATUS_RUNNING
        self.frm_status_bar.setStyleSheet(
            f"QFrame {{ background-color: {color}; border: none; }}"
        )

    def _refresh_button(self):
        icon = Icons.STOP() if self.is_running else Icons.RUN()
        self.btn_run_stop.setIcon(icon)
        self.btn_run_stop.setIconSize(Layout.ICON_SIZE_MD)

    def _refresh_progress(self):
        if self.is_running:
            self.progress_bar.show()
            self.test_result_label.hide()
        else:
            self.progress_bar.hide()
            self.test_result_label.show()
            self.test_result_label.setText(self.test_result)

    def _refresh_icon(self):
        self.status_icon.setPixmap(
            Icons.for_status(self.test_status, self.is_running)
            .pixmap(Layout.ICON_SIZE_MD)
        )

    def _setup_styles(self):
        self.setStyleSheet(
            f"QWidget {{ background-color: transparent; }}"
        )
        self.btn_run_stop.setStyleSheet(Styles.BUTTON_ICON)
        self.progress_bar.setStyleSheet(Styles.PROGRESS_BAR)
        self.status_icon.setStyleSheet("background-color: transparent;")
        self.lbl_test_name.setStyleSheet(
            f"color: {Colors.TEXT_PRIMARY}; font-weight: bold; font-size: 13px;"
            f" background-color: transparent;"
        )
        self.test_result_label.setStyleSheet(
            f"color: {Colors.TEXT_SECONDARY}; font-size: 11px;"
            f" background-color: transparent;"
        )
