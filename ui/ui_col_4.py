from datetime import datetime

from PyQt5.QtWidgets import QWidget, QApplication, QFileDialog, QScrollArea, QVBoxLayout
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

from .theme import Colors, Styles, Icons, Layout, QT_DIR


class ColCapture(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_capture.ui", self)
        self._setup_styles()
        self._connect_signals()

    def append_log(self, text: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        current   = self.lbl_log.text()
        line      = f"[{timestamp}] {text}"
        self.lbl_log.setText((current + "\n" + line).lstrip())
        self.lbl_log.repaint()

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
                background-color: {Colors.BG_TOOLBAR};
                border-top: 1px solid {Colors.BORDER};
            }}
            QLabel#lbl_capture_image {{
                background-color: {Colors.BG_IMAGE};
                color: {Colors.TEXT_MUTED};
                font-size: 12px;
            }}
            QLabel#lbl_log {{
                background-color: {Colors.BG_LOG};
                color: {Colors.TEXT_PRIMARY};
                font-family: monospace;
                font-size: 11px;
                padding: 8px;
            }}
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
