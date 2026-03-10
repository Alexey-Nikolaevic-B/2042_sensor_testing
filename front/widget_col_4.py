from datetime import datetime

from PyQt5.QtWidgets import QWidget, QApplication
from PyQt5.QtCore import Qt, QMetaObject, Q_ARG, pyqtSlot
from PyQt5.QtGui import QColor, QTextCharFormat, QTextCursor
from PyQt5 import uic

from ._theme import Colors, Styles, Icons, Layout, QT_DIR
from .widget_col_2 import TOOLBAR_H


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
        from PyQt5.QtCore import QThread
        if QThread.currentThread() is not self.thread():
            QMetaObject.invokeMethod(
                self, "_append_log_main",
                Qt.QueuedConnection,
                Q_ARG(str, level),
                Q_ARG(str, source),
                Q_ARG(str, message),
            )
            return
        self._append_log_main(level, source, message)

    @pyqtSlot(str, str, str)
    def _append_log_main(self, level: str, source: str, message: str) -> None:
        prefix, color = _LEVEL_FMT.get(level.lower(), ("[INFO]", "#9ca3af"))
        ts = datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}]  {prefix}  {source}: {message}"
        self._append_colored(line, color)

    def _on_clear(self):
        self.log_view.clear()

    def _on_copy(self):
        text = self.log_view.toPlainText()
        if text:
            QApplication.clipboard().setText(text)

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
        self.wt_toolbar_bottom.setFixedHeight(TOOLBAR_H)

    def _connect_signals(self):
        self.btn_clear_log.clicked.connect(self._on_clear)
        self.btn_copy_log.clicked.connect(self._on_copy)

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QWidget {{ background-color: {Colors.BG_COLUMN}; }}
            QWidget#wt_toolbar_bottom {{
                background-color: {Colors.BG_LOG};
                border-top: 1px solid {Colors.BORDER};
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
        for btn, icon in [
            (self.btn_clear_log, Icons.CLEAR()),
            (self.btn_copy_log,  Icons.COPY()),
        ]:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(Styles.BUTTON_ICON)