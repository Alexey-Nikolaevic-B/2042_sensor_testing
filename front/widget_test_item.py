import os
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5 import uic

from ._theme import Styles, Icons, Layout, Colors, QT_DIR
from .logic_queue_manager import TestStatus


class TestItem(QWidget):
    run_requested  = pyqtSignal(str)
    stop_requested = pyqtSignal(str)
    selected       = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(os.path.join(QT_DIR, "test_item.ui"), self)

        self.func_name        = ""
        self.test_name        = ""
        self.test_description = ""
        self.test_result      = ""
        self._image_path      = ""
        self._progress        = 0
        self._movie           = None
        self.is_selected      = False
        self.test_status      = TestStatus.IDLE

        self.setObjectName("TestItem")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setCursor(Qt.PointingHandCursor)
        self._setup_styles()
        self.btn_run_stop.clicked.connect(self._on_run_stop_clicked)

    _DB_STATUS_MAP = {
        "Passed":  TestStatus.PASSED,
        "Failed":  TestStatus.FAILED,
        "Stopped": TestStatus.IDLE,
        "Pending": TestStatus.IDLE,
        "Idle":    TestStatus.IDLE,
    }

    def load(self, test_data: dict):
        self._stop_movie()
        self.func_name        = test_data.get("name", "")
        self.test_name        = test_data.get("display_name") or self.func_name
        self.test_description = test_data.get("description", "")
        self._image_path      = test_data.get("image_path", "")
        self._progress        = 0
        self.is_selected      = False

        db_status  = test_data.get("status", "Pending")
        db_result  = test_data.get("result", "")
        self.test_status = self._DB_STATUS_MAP.get(db_status, TestStatus.IDLE)

        if isinstance(db_result, dict):
            parts = [f"{k}: {v}" for k, v in db_result.items()
                     if k not in ("passed", "duration")]
            self.test_result = "  |  ".join(parts)
        else:
            self.test_result = str(db_result) if db_result else ""

        self._refresh_all()

    def set_status(self, status: TestStatus) -> None:
        old = self.test_status
        self.test_status = status

        # Gif: pending.gif while QUEUED, running.gif while RUNNING
        if status == TestStatus.QUEUED:
            self._start_movie(Icons.QUEUED)
        elif status == TestStatus.RUNNING:
            self._start_movie(Icons.RUNNING_MOVIE)
        elif old in (TestStatus.QUEUED, TestStatus.RUNNING):
            self._stop_movie()

        # Reset progress when leaving RUNNING
        if old == TestStatus.RUNNING and status != TestStatus.RUNNING:
            self._progress = 0

        self._refresh_all()

    def set_progress(self, value: int) -> None:
        if self.test_status != TestStatus.RUNNING:
            return
        self._progress = max(0, min(100, value))
        self.progress_bar.setValue(self._progress)

    def set_result(self, result: dict) -> None:
        if not isinstance(result, dict):
            return
        skip = {"passed", "duration"}
        parts = [f"{k}: {v}" for k, v in result.items() if k not in skip]
        self.test_result = "  |  ".join(parts) if parts else ""
        self.test_result_label.setText(self.test_result)

    def set_selected(self, selected: bool):
        self.is_selected = selected
        self._refresh_bg()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.selected.emit(self.func_name)
        super().mousePressEvent(event)

    def enterEvent(self, event):
        if not self.is_selected:
            self.setStyleSheet(self._bg_style(Colors.BG_CARD_HOVER))
        super().enterEvent(event)

    def leaveEvent(self, event):
        if not self.is_selected:
            self.setStyleSheet(self._bg_style(Colors.BG_CARD))
        super().leaveEvent(event)

    def _on_run_stop_clicked(self):
        if self.test_status in (TestStatus.QUEUED, TestStatus.RUNNING):
            self.stop_requested.emit(self.func_name)
        else:
            self.run_requested.emit(self.func_name)

    def _start_movie(self, factory):
        self._stop_movie()
        self._movie = factory(self.status_icon)
        self.status_icon.setMovie(self._movie)
        self._movie.start()

    def _stop_movie(self):
        if self._movie is not None:
            self._movie.stop()
            self._movie = None
            self.status_icon.setMovie(None)

    def _refresh_all(self):
        self.lbl_test_name.setText(self.test_name)
        self._refresh_status_bar()
        self._refresh_button()
        self._refresh_progress()
        self._refresh_icon()
        self._refresh_bg()

    def _refresh_bg(self):
        bg = Colors.BG_CARD_SEL if self.is_selected else Colors.BG_CARD
        self.setStyleSheet(self._bg_style(bg))

    def _refresh_status_bar(self):
        color = {
            TestStatus.IDLE:    Colors.STATUS_GRAY,
            TestStatus.QUEUED:  Colors.STATUS_QUEUED,
            TestStatus.RUNNING: Colors.STATUS_RUNNING,
            TestStatus.PASSED:  Colors.STATUS_GREEN,
            TestStatus.FAILED:  Colors.STATUS_RED,
        }.get(self.test_status, Colors.STATUS_BLUE)
        self.frm_status_bar.setStyleSheet(
            f"QFrame {{ background-color: {color}; border: none; }}"
        )

    def _refresh_button(self):
        active = self.test_status in (TestStatus.QUEUED, TestStatus.RUNNING)
        self.btn_run_stop.setIcon(Icons.STOP() if active else Icons.RUN())
        self.btn_run_stop.setIconSize(Layout.ICON_SIZE_MD)

    def _refresh_progress(self):
        if self.test_status == TestStatus.RUNNING:
            self.progress_bar.setRange(0, 100)
            self.progress_bar.setValue(self._progress)
            self.progress_bar.show()
            self.test_result_label.hide()
        elif self.test_status == TestStatus.IDLE:
            self.progress_bar.hide()
            self.test_result_label.show()
            self.test_result_label.setText("")
        else:
            self.progress_bar.hide()
            self.test_result_label.show()
            self.test_result_label.setText(self.test_result)

    def _refresh_icon(self):
        if self.test_status in (TestStatus.QUEUED, TestStatus.RUNNING):
            return  # movie owns the label
        is_running = False
        self.status_icon.setPixmap(
            Icons.for_status(self.test_status.value, is_running)
            .pixmap(Layout.ICON_SIZE_MD)
        )

    @staticmethod
    def _bg_style(bg: str) -> str:
        return (
            f"QWidget#TestItem {{ background-color: {bg};"
            f" border-bottom: 1px solid {Colors.DIVIDER}; }}"
        )

    def _setup_styles(self):
        self.setStyleSheet(self._bg_style(Colors.BG_CARD))
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