import os

from PyQt5.QtWidgets import QWidget, QApplication, QLabel, QFrame, QHBoxLayout, QSizePolicy
from PyQt5.QtCore import Qt, QMetaObject, Q_ARG, pyqtSlot, pyqtSignal
from PyQt5.QtGui import QColor, QTextCharFormat, QTextCursor, QPixmap, QImage
from PyQt5 import uic

from ._theme import Colors, Styles, Icons, Layout, QT_DIR

_LEVEL_FMT: dict[str, tuple[str, str]] = {
    "debug":    ("[DEBG]", "#6b7280"),
    "info":     ("[INFO]", "#9ca3af"),
    "warning":  ("[WARN]", "#f59e0b"),
    "error":    ("[ERRO]", "#ef4444"),
    "critical": ("[CRIT]", "#dc2626"),
}


_fmt_value = lambda v: "\n".join(f"- {x}" for x in v) if isinstance(v, list) else str(v)


class ColCapture(QWidget):

    _capture_arrived = pyqtSignal(dict, bytes)

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_4.ui", self)

        self._simulator         = None
        self._show_observer     = False
        self._last_sensor_data: dict       = {}
        self._last_obs_img:     bytes|None = None

        self._setup_heights()
        self._setup_styles()
        self._connect_signals()
        self._capture_arrived.connect(self._on_capture_main)
        # Give the result panel a sensible default split; user can resize freely
        self.splitter_result_log.setSizes([150, 300])
        # Image pane = IMAGE_H minus the 3px splitter handle so its bottom
        # aligns with the image labels in col_2 and col_3
        self.splitter_image_bottom.setSizes([Layout.IMAGE_H - 3, 600])

    # ── public API ────────────────────────────────────────────────────────────

    def load_test_result(self, func_name: str, result: dict) -> None:
        """Replace the result panel contents with the latest test result."""
        self._clear_result_panel()
        layout = self.scroll_test_result_contents.layout()

        if not result:
            lbl = QLabel("no result yet")
            lbl.setStyleSheet(
                f"color: {Colors.TEXT_MUTED}; font-size: 12px;"
                f" padding: 10px; background: transparent;"
            )
            layout.addWidget(lbl)
        else:
            for key, value in result.items():
                row = QFrame()
                row.setStyleSheet(
                    f"QFrame {{ border-bottom: 1px solid {Colors.BORDER};"
                    f" background: transparent; }}"
                )
                h = QHBoxLayout(row)
                h.setContentsMargins(12, 6, 12, 6)
                h.setSpacing(12)

                lbl_key = QLabel(str(key))
                lbl_key.setStyleSheet(
                    f"color: {Colors.TEXT_SECONDARY}; font-size: 12px;"
                    f" min-width: 110px; max-width: 110px;"
                    f" background: transparent; border: none;"
                )

                lbl_val = QLabel(_fmt_value(value))
                lbl_val.setStyleSheet(
                    f"color: {Colors.TEXT_PRIMARY}; font-size: 12px;"
                    f" background: transparent; border: none;"
                )
                lbl_val.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
                lbl_val.setWordWrap(True)
                lbl_val.setTextInteractionFlags(Qt.TextSelectableByMouse)

                h.addWidget(lbl_key)
                h.addWidget(lbl_val)
                layout.addWidget(row)

        filler = QWidget()
        filler.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        filler.setStyleSheet("background-color: transparent;")
        layout.addWidget(filler)

    def _clear_result_panel(self) -> None:
        layout = self.scroll_test_result_contents.layout()
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

    def set_simulator(self, simulator) -> None:
        self._simulator = simulator
        simulator.on_capture = self._on_capture_from_thread
        # When the simulator blocks on wait_for_step, it calls this to enable btn_step
        simulator.on_waiting_for_step = self._on_sim_waiting_for_step

    # ── lock / step ───────────────────────────────────────────────────────────

    def _on_lock_toggled(self, checked: bool) -> None:
        if self._simulator:
            self._simulator.set_step_mode(checked)
        if not checked:
            # Unlocking — release any pending gate and disable step button
            self.btn_step.setEnabled(False)
            if self._simulator:
                self._simulator.continue_all()

    def _on_step_clicked(self) -> None:
        self.btn_step.setEnabled(False)
        if self._simulator:
            self._simulator.advance_step()

    @pyqtSlot()
    def _on_sim_waiting_for_step(self) -> None:
        """Called from simulator (via invokeMethod) when it's blocked on wait_for_step."""
        self.btn_step.setEnabled(True)

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
        self._capture_arrived.emit(sensor_data, obs_img or b"")

    @pyqtSlot(dict, bytes)
    def _on_capture_main(self, sensor_data: dict, obs_img: bytes) -> None:
        """Runs on main thread — safe to update widgets."""
        self._last_sensor_data = sensor_data
        self._last_obs_img     = obs_img if obs_img else None
        self._refresh_display()

    # ── display ───────────────────────────────────────────────────────────────

    def _refresh_display(self) -> None:
        if self._show_observer:
            self._show_image_bytes(self._last_obs_img, "no observer frame")
        else:
            self._show_sensor_data(self._last_sensor_data)

    def _show_sensor_data(self, data: dict) -> None:
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
        if data:
            img = QImage.fromData(data)
            if not img.isNull():
                px = QPixmap.fromImage(img).scaled(
                    self.lbl_capture_image.width(),
                    self.lbl_capture_image.height(),
                    Qt.KeepAspectRatio, Qt.SmoothTransformation,
                )
                self.lbl_capture_image.setPixmap(px)
                self.lbl_capture_image.setText("")
                return
        self._clear_display(fallback)

    def _clear_display(self, text: str = "no capture data") -> None:
        self.lbl_capture_image.setPixmap(QPixmap())
        self.lbl_capture_image.setText(text)

    # ── slots — view toggle ───────────────────────────────────────────────────

    def _on_view_sensor(self) -> None:
        self._show_observer = False
        self.btn_view_sensor.setChecked(True)
        self.btn_view_observer.setChecked(False)
        self._refresh_display()

    def _on_view_observer(self) -> None:
        self._show_observer = True
        self.btn_view_observer.setChecked(True)
        self.btn_view_sensor.setChecked(False)
        self._refresh_display()

    # ── slots — log toolbar ───────────────────────────────────────────────────

    def _on_clear(self):
        self.log_view.clear()

    def _on_copy(self):
        text = self.log_view.toPlainText()
        if text:
            QApplication.clipboard().setText(text)

    # ── log ───────────────────────────────────────────────────────────────────

    @pyqtSlot(str, str, str)
    def _append_log_main(self, level: str, source: str, message: str) -> None:
        prefix, color = _LEVEL_FMT.get(level.lower(), ("[INFO]", "#9ca3af"))
        line = f"{prefix}  {source}: {message}"
        self._append_colored(line, color)

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

    def _setup_heights(self):
        self.lbl_capture_image.setTextFormat(Qt.RichText)
        self.lbl_capture_image.setAlignment(Qt.AlignCenter)
        self.lbl_capture_image.setWordWrap(True)
        self.wt_toolbar_capture.setFixedHeight(Layout.TOOLBAR_H)
        self.wt_toolbar_bottom.setFixedHeight(Layout.TOOLBAR_H)

    def _connect_signals(self):
        self.btn_view_sensor.clicked.connect(self._on_view_sensor)
        self.btn_view_observer.clicked.connect(self._on_view_observer)
        self.btn_clear_log.clicked.connect(self._on_clear)
        self.btn_copy_log.clicked.connect(self._on_copy)
        self.btn_lock.toggled.connect(self._on_lock_toggled)
        self.btn_step.clicked.connect(self._on_step_clicked)

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QWidget {{ background-color: {Colors.BG_COLUMN}; }}
            QLabel#lbl_capture_image {{
                background-color: {Colors.BG_IMAGE};
                color: {Colors.TEXT_MUTED};
                font-size: 12px;
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
            QSplitter#splitter_image_bottom::handle {{
                background-color: {Colors.BORDER};
                height: 3px;
            }}
            QSplitter#splitter_image_bottom::handle:hover {{
                background-color: {Colors.ACCENT};
            }}
            QSplitter#splitter_result_log::handle {{
                background-color: {Colors.BORDER};
                height: 3px;
            }}
            QSplitter#splitter_result_log::handle:hover {{
                background-color: {Colors.ACCENT};
            }}
            QScrollArea#scroll_test_result {{
                border: none;
                background-color: transparent;
                border-bottom: 1px solid {Colors.BORDER};
            }}
            QWidget#scroll_test_result_contents {{
                background-color: transparent;
            }}
            {Styles.SCROLLBAR}
        """)
        for btn, icon in [
            (self.btn_lock,          Icons.LOCK()),
            (self.btn_step,          Icons.STEP()),
            (self.btn_view_sensor,   Icons.TARGET_SENSOR()),
            (self.btn_view_observer, Icons.OBSERVER()),
            (self.btn_clear_log,     Icons.CLEAR()),
            (self.btn_copy_log,      Icons.COPY()),
        ]:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(Styles.BUTTON_ICON)

        # Override lock button to show checked state
        self.btn_lock.setStyleSheet(Styles.BUTTON_ICON + f"""
            QPushButton:checked {{
                background-color: {Colors.ACCENT_DIM};
                border: 1px solid {Colors.ACCENT};
                border-radius: 4px;
            }}
        """)