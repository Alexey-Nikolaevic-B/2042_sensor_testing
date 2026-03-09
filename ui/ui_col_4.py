from PyQt5.QtWidgets import QWidget, QApplication, QFileDialog
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QColor, QTextCharFormat, QTextCursor
from PyQt5 import uic

from .theme import Colors, Styles, Icons, Layout, QT_DIR
from .ui_col_2 import IMAGE_H, TOOLBAR_H


_LEVEL_FMT: dict[str, tuple[str, str]] = {
    "debug":    ("[DEBG]", "#6b7280"),
    "info":     ("[INFO]", "#9ca3af"),
    "warning":  ("[WARN]", "#f59e0b"),
    "error":    ("[ERRO]", "#ef4444"),
    "critical": ("[CRIT]", "#dc2626"),
}



class ColCapture(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_4.ui", self)
        self._enforce_heights()
        self._setup_styles()
        self._connect_signals()

    def append_log(self, level: str, source: str, message: str) -> None:
        prefix, color = _LEVEL_FMT.get(level.lower(), ("INF", "#9ca3af"))
        line = f"{prefix}  {source}: {message}"
        self._append_colored(line, color)

    def set_capture_image(self, pixmap: QPixmap) -> None:
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
        self.log_view.clear()

    def _on_copy(self):
        text = self.log_view.toPlainText()
        if text:
            QApplication.clipboard().setText(text)

    def _on_save(self):
        text = self.log_view.toPlainText()
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
                self.append_log("error", "ui.col_4", f"Error saving log: {e}")

    def _append_colored(self, text: str, hex_color: str) -> None:
        fmt = QTextCharFormat()
        fmt.setForeground(QColor(hex_color))
        cursor = self.log_view.textCursor()
        cursor.movePosition(QTextCursor.End)
        if self.log_view.toPlainText():
            cursor.insertText("\n")
        cursor.insertText(text, fmt)
        self.log_view.setTextCursor(cursor)
        self.log_view.ensureCursorVisible()

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
            QPlainTextEdit#log_view {{
                background-color: {Colors.BG_LOG};
                color: {Colors.TEXT_PRIMARY};
                font-family: monospace;
                font-size: 11px;
                border: none;
                padding: 6px;
                selection-background-color: #334155;
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