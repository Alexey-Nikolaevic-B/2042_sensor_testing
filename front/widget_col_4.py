import os
from datetime import datetime

from PyQt5.QtWidgets import QWidget, QApplication
from PyQt5.QtCore import Qt, QMetaObject, Q_ARG, pyqtSlot, pyqtSignal
from PyQt5.QtGui import QColor, QTextCharFormat, QTextCursor, QPixmap, QImage, QIcon
from PyQt5 import uic

from ._theme import Colors, Styles, Icons, Layout, QT_DIR
from .widget_col_2 import TOOLBAR_H

_ICON_DIR = os.path.join(os.path.dirname(__file__), "icon")

IMAGE_H = 220

_LEVEL_FMT: dict[str, tuple[str, str]] = {
    "debug":    ("[DEBG]", "#6b7280"),
    "info":     ("[INFO]", "#9ca3af"),
    "warning":  ("[WARN]", "#f59e0b"),
    "error":    ("[ERRO]", "#ef4444"),
    "critical": ("[CRIT]", "#dc2626"),
}


def _icon(name: str) -> QIcon:
    path = os.path.join(_ICON_DIR, name)
    return QIcon(path) if os.path.exists(path) else QIcon()


class ColCapture(QWidget):

    # Signal fired from test thread — routed to main thread safely
    _capture_arrived = pyqtSignal(dict, bytes)

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_4.ui", self)

        self._simulator         = None
        self._show_observer     = False
        self._last_sensor_data: dict       = {}
        self._last_obs_img:     bytes|None = None

        self._enforce_heights()
        self._setup_styles()
        self._connect_signals()
        self._capture_arrived.connect(self._on_capture_main)

    # ── public API ────────────────────────────────────────────────────────────

    def set_simulator(self, simulator) -> None:
        print(f"[DEBUG col_4] set_simulator called: {simulator}")
        self._simulator = simulator
        simulator.on_capture = self._on_capture_from_thread
        print(f"[DEBUG col_4] simulator.on_capture wired to: {simulator.on_capture}")

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

    # ── capture callbacks ─────────────────────────────────────────────────────

    def _on_capture_from_thread(self, sensor_data: dict, obs_img: bytes | None) -> None:
        """Called from test worker thread — must NOT touch Qt widgets directly."""
        import threading
        print(f"[DEBUG col_4] _on_capture_from_thread: "
              f"thread={threading.current_thread().name} "
              f"sensor_data={sensor_data} "
              f"obs_img_len={len(obs_img) if obs_img else 0}")
        self._capture_arrived.emit(sensor_data, obs_img or b"")
        print("[DEBUG col_4] _capture_arrived.emit done")

    @pyqtSlot(dict, bytes)
    def _on_capture_main(self, sensor_data: dict, obs_img: bytes) -> None:
        """Runs on main thread — safe to update widgets."""
        print(f"[DEBUG col_4] _on_capture_main: "
              f"sensor_data={sensor_data} "
              f"obs_img_len={len(obs_img)} "
              f"show_observer={self._show_observer}")
        self._last_sensor_data = sensor_data
        self._last_obs_img     = obs_img if obs_img else None
        self._refresh_display()

    # ── image / data display ──────────────────────────────────────────────────

    def _refresh_display(self) -> None:
        print(f"[DEBUG col_4] _refresh_display: show_observer={self._show_observer}")
        if self._show_observer:
            self._show_image_bytes(self._last_obs_img, "no observer frame")
        else:
            self._show_sensor_data(self._last_sensor_data)

    def _show_sensor_data(self, data: dict) -> None:
        print(f"[DEBUG col_4] _show_sensor_data: data={data}")
        if not data:
            self._clear_display()
            return
        lines = [
            f"<b>{data.get('sensor_name', '')} · {data.get('sensor_type', '')}</b>",
            f"topic: {data.get('topic', '')}",
            f"frames captured: <b>{data.get('count', 0)}</b>",
        ]
        frames = data.get("frames", [])
        if frames:
            lines.append("ids: " + ", ".join(str(f) for f in frames[:8]))
            if len(frames) > 8:
                lines.append(f"… and {len(frames) - 8} more")
        self.lbl_capture_image.setPixmap(QPixmap())
        self.lbl_capture_image.setText("<br>".join(lines))

    def _show_image_bytes(self, data: bytes | None, fallback: str) -> None:
        print(f"[DEBUG col_4] _show_image_bytes: data_len={len(data) if data else 0} fallback={fallback!r}")
        if data:
            img = QImage.fromData(data)
            print(f"[DEBUG col_4] QImage.fromData: null={img.isNull()} size={img.width()}x{img.height()}")
            if not img.isNull():
                px = QPixmap.fromImage(img).scaled(
                    self.lbl_capture_image.width(), IMAGE_H,
                    Qt.KeepAspectRatio, Qt.SmoothTransformation,
                )
                print(f"[DEBUG col_4] pixmap: {px.width()}x{px.height()} null={px.isNull()}")
                self.lbl_capture_image.setPixmap(px)
                self.lbl_capture_image.setText("")
                return
        self._clear_display(fallback)

    def _clear_display(self, text: str = "no capture data") -> None:
        self.lbl_capture_image.setPixmap(QPixmap())
        self.lbl_capture_image.setText(text)

    # ── view toggle ───────────────────────────────────────────────────────────

    def _on_view_sensor(self) -> None:
        print("[DEBUG col_4] _on_view_sensor clicked")
        self._show_observer = False
        self.btn_view_sensor.setChecked(True)
        self.btn_view_observer.setChecked(False)
        self._refresh_display()

    def _on_view_observer(self) -> None:
        print("[DEBUG col_4] _on_view_observer clicked")
        self._show_observer = True
        self.btn_view_observer.setChecked(True)
        self.btn_view_sensor.setChecked(False)
        self._refresh_display()

    # ── log ───────────────────────────────────────────────────────────────────

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

    # ── setup ─────────────────────────────────────────────────────────────────

    def _enforce_heights(self):
        self.lbl_capture_image.setFixedHeight(IMAGE_H)
        self.lbl_capture_image.setTextFormat(Qt.RichText)
        self.lbl_capture_image.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self.lbl_capture_image.setWordWrap(True)
        self.wt_toolbar_capture.setFixedHeight(TOOLBAR_H)
        self.wt_toolbar_bottom.setFixedHeight(TOOLBAR_H)

    def _connect_signals(self):
        self.btn_view_sensor.clicked.connect(self._on_view_sensor)
        self.btn_view_observer.clicked.connect(self._on_view_observer)
        self.btn_clear_log.clicked.connect(self._on_clear)
        self.btn_copy_log.clicked.connect(self._on_copy)

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QWidget {{ background-color: {Colors.BG_COLUMN}; }}
            QLabel#lbl_capture_image {{
                background-color: {Colors.BG_IMAGE};
                color: {Colors.TEXT_PRIMARY};
                font-size: 12px;
                padding: 10px;
            }}
            QWidget#wt_toolbar_capture {{
                background-color: {Colors.BG_TOOLBAR};
                border-top: 1px solid {Colors.BORDER};
                border-bottom: 1px solid {Colors.BORDER};
            }}
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
            QPushButton:checked {{
                background-color: {Colors.ACCENT_DIM};
                border: 1px solid {Colors.ACCENT};
            }}
            {Styles.SCROLLBAR}
        """)

        for btn, icon_file in [
            (self.btn_view_sensor,   "target_sensor.png"),
            (self.btn_view_observer, "observer.png"),
        ]:
            btn.setIcon(_icon(icon_file))
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(Styles.BUTTON_ICON)

        for btn, icon in [
            (self.btn_clear_log, Icons.CLEAR()),
            (self.btn_copy_log,  Icons.COPY()),
        ]:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(Styles.BUTTON_ICON)