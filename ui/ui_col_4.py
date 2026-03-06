from datetime import datetime

from PyQt5.QtWidgets import QWidget, QApplication, QFileDialog, QLabel, QScrollArea, QVBoxLayout, QSizePolicy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

from .theme import Colors, Styles, Icons, Layout, QT_DIR
from .ui_col_2 import IMAGE_H, TOOLBAR_H


class ColCapture(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_capture.ui", self)
        self._enforce_heights()
        self._setup_styles()
        self._connect_signals()

    def append_log(self, text: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        current   = self.lbl_log.text()
        line      = f"[{timestamp}] {text}"
        self.lbl_log.setText((current + "\n" + line).lstrip())
        QTimer.singleShot(0, self._scroll_to_bottom)

    def set_capture_image(self, pixmap: QPixmap):
        if pixmap and not pixmap.isNull():
            self.lbl_capture_image.setPixmap(
                pixmap.scaled(
                    self.lbl_capture_image.width(),
                    self.lbl_capture_image.height(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
            )
        else:
            self.lbl_capture_image.clear()
            self.lbl_capture_image.setText("no data captured")

    def _on_clear(self):
        self.lbl_log.setText("")

    def _on_copy(self):
        text = self.lbl_log.text()
        if text:
            QApplication.clipboard().setText(text)

    def _on_save(self):
        text = self.lbl_log.text()
        if not text:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Log",
            f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
            "Text Files (*.txt);;All Files (*)",
        )
        if path:
            try:
                with open(path, "w") as f:
                    f.write(text)
            except OSError as e:
                self.append_log(f"Error saving: {e}")

    def _scroll_to_bottom(self):
        sb = self.scroll_log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _enforce_heights(self):
        self.lbl_capture_image.setFixedHeight(IMAGE_H)
        self.wt_toolbar_top.setFixedHeight(TOOLBAR_H)
        self.wt_toolbar_bottom.setFixedHeight(TOOLBAR_H)

    def _connect_signals(self):
        self.btn_clear_log.clicked.connect(self._on_clear)
        self.btn_copy_log.clicked.connect(self._on_copy)
        self.btn_save_capture.clicked.connect(self._on_save)

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QWidget {{ background-color: {Colors.BG_COLUMN}; }}
            QWidget#wt_toolbar_top {{
                background-color: {Colors.BG_TOOLBAR};
                border-bottom: 1px solid {Colors.BORDER};
            }}
            QWidget#wt_toolbar_bottom {{
                background-color: {Colors.BG_LOG};
                border-top: 1px solid {Colors.BORDER};
            }}
            QLabel#lbl_capture_image {{
                background-color: {Colors.BG_IMAGE};
                color: {Colors.TEXT_MUTED};
                font-size: 12px;
            }}
            QScrollArea#scroll_log {{
                border: none;
                background-color: {Colors.BG_LOG};
            }}
            QWidget#scroll_log_contents {{
                background-color: {Colors.BG_LOG};
            }}
            QLabel#lbl_log {{
                background-color: {Colors.BG_LOG};
                color: {Colors.TEXT_PRIMARY};
                font-family: monospace;
                font-size: 11px;
                padding: 8px;
            }}
            {Styles.SCROLLBAR}
        """)
        self.btn_capture.setStyleSheet(Styles.BUTTON_ACCENT)
        for btn, icon in [
            (self.btn_save_capture, Icons.SAVE()),
            (self.btn_clear_log,    Icons.CLEAR()),
            (self.btn_copy_log,     Icons.COPY()),
        ]:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(Styles.BUTTON_ICON)