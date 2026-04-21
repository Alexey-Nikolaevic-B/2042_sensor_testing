import os

from PyQt5.QtWidgets import (
    QWidget,
    QApplication,
    QLabel,
    QFrame,
    QHBoxLayout,
    QSizePolicy,
)
from PyQt5.QtCore import Qt, QMetaObject, Q_ARG, pyqtSlot, pyqtSignal
from PyQt5.QtGui import QColor, QTextCharFormat, QTextCursor, QPixmap, QImage
from PyQt5 import uic

from ._theme import Colors, Styles, Icons, Layout, QT_DIR

_LEVEL_FMT: dict[str, tuple[str, str]] = {
    "debug": ("[DEBG]", "#6b7280"),
    "info": ("[INFO]", "#9ca3af"),
    "warning": ("[WARN]", "#f59e0b"),
    "error": ("[ERRO]", "#ef4444"),
    "critical": ("[CRIT]", "#dc2626"),
}


def _fmt_value(v):
    if isinstance(v, list):
        return "\n".join(f"- {x}" for x in v)
    if isinstance(v, dict):
        lines = []
        for k, val in v.items():
            if isinstance(val, dict):
                inner = ", ".join(f"{ik}: {iv}" for ik, iv in val.items())
                lines.append(f"{k}: {{{inner}}}")
            elif isinstance(val, list):
                lines.append(f"{k}: [{', '.join(str(x) for x in val)}]")
            else:
                lines.append(f"{k}: {val}")
        return "\n".join(lines)
    return str(v)


class ColCapture(QWidget):

    _capture_arrived = pyqtSignal(dict, bytes)
    _log_arrived = pyqtSignal(str, str, str)  # level, source, message
    _sep_arrived = pyqtSignal(str)  # separator label

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_4.ui", self)

        self._simulator = None
        self._show_observer = False
        self._last_sensor_data: dict = {}
        self._last_obs_img: bytes | None = None
        self._last_sensor_img: bytes | None = None
        self._image_too_large = False
        self._image_too_large_resolution = ""

        self._setup_heights()
        self._setup_styles()
        self._connect_signals()
        self._capture_arrived.connect(self._on_capture_main)
        self._log_arrived.connect(self._append_log_main)
        self._sep_arrived.connect(self._append_separator_main)
        # Give the result panel a sensible default split; user can resize freely
        self.splitter_result_log.setSizes([150, 300])
        # Image pane = IMAGE_H minus the 3px splitter handle so its bottom
        # aligns with the image labels in col_2 and col_3
        self.splitter_image_bottom.setSizes([Layout.IMAGE_H - 3, 600])

    # ── public API ────────────────────────────────────────────────────────────

    def clear_capture(self) -> None:
        """Called when a new test starts — reset the image area."""
        self._last_sensor_data = {}
        self._last_obs_img = None
        self._last_sensor_img = None
        self._clear_display()

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
            # Flatten result: top-level keys shown as rows;
            # if value is a dict (e.g. "metrics"), expand its children
            # as separate rows with indented keys.
            flat_items = []
            # Put "description" first if it exists
            if "description" in result:
                flat_items.append(("description", result["description"]))
            for key, value in result.items():
                if key == "description":
                    continue  # already added above
                if isinstance(value, dict) and key in ("metrics", "checks", "diagnostics"):
                    for sub_key, sub_val in value.items():
                        # Skip bulky nested lists/dicts that clutter the UI
                        # (e.g. "measurements": [{...}, {...}], "samples": [...])
                        if isinstance(sub_val, list) and sub_val and isinstance(sub_val[0], dict):
                            flat_items.append((f"  {sub_key}", f"[{len(sub_val)} items]"))
                        elif isinstance(sub_val, dict) and len(str(sub_val)) > 200:
                            flat_items.append((f"  {sub_key}", f"{{{len(sub_val)} keys}}"))
                        else:
                            flat_items.append((f"  {sub_key}", sub_val))
                else:
                    flat_items.append((key, value))

            for key, value in flat_items:
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
        self._log_arrived.emit(level, source, message)

    # ── capture callbacks ─────────────────────────────────────────────────────

    def _on_capture_from_thread(self, sensor_data: dict, obs_img: bytes | None) -> None:
        """Called from test worker thread — must NOT touch Qt widgets directly."""
        self._capture_arrived.emit(sensor_data, obs_img or b"")

    @pyqtSlot(dict, bytes)
    def _on_capture_main(self, sensor_data: dict, obs_img: bytes) -> None:
        """Runs on main thread — safe to update widgets."""
        self._last_sensor_data = sensor_data
        self._last_obs_img = obs_img if obs_img else None
        # Render sensor image from messages on capture arrival
        self._last_sensor_img = self._render_sensor_image(sensor_data)
        self._refresh_display()

    # ── display ───────────────────────────────────────────────────────────────

    def _refresh_display(self) -> None:
        if self._show_observer:
            self._show_image_bytes(self._last_obs_img, "no observer frame")
        else:
            if self._last_sensor_img:
                self._show_image_bytes(self._last_sensor_img, "no sensor image")
            elif getattr(self, "_image_too_large", False):
                res = getattr(self, "_image_too_large_resolution", "?")
                self._show_placeholder(
                    f"Изображение {res} слишком большое для отображения в UI.\n"
                    f"Тест выполняется в фоновом режиме без вывода кадров."
                )
            elif self._last_sensor_data:
                self._show_static_image(self._last_sensor_data.get("image_path", ""))
            else:
                self._clear_display()

    # Max resolution for UI rendering (width * height).
    # Images above this are skipped to prevent OOM crashes.
    _MAX_RENDER_PIXELS = 1920 * 1080

    def _render_sensor_image(self, data: dict) -> bytes | None:
        """Convert raw ROS messages to JPEG bytes for display."""
        import logging as _log

        _dbg = _log.getLogger(__name__)

        msgs = data.get("messages", [])
        _dbg.warning(
            "[col4] render: sensor_type=%s msgs=%d image_path=%r",
            data.get("sensor_type"),
            len(msgs),
            data.get("image_path"),
        )
        if not msgs:
            _dbg.warning("[col4] render: no messages — returning None")
            return None
        msg = msgs[-1]

        # Skip rendering for very large images to prevent OOM
        w = getattr(msg, "width", 0) or 0
        h = getattr(msg, "height", 0) or 0
        if w * h > self._MAX_RENDER_PIXELS:
            _dbg.warning(
                "[col4] render: image too large (%dx%d = %d px), skipping to prevent OOM",
                w, h, w * h,
            )
            self._image_too_large = True
            self._image_too_large_resolution = f"{w}x{h}"
            return None
        self._image_too_large = False

        _dbg.warning(
            "[col4] render: msg type=%s attrs=%s",
            type(msg).__name__,
            [
                a
                for a in ("encoding", "height", "width", "data", "states")
                if hasattr(msg, a)
            ],
        )

        try:
            # ── Camera: sensor_msgs/Image ─────────────────────────────────
            if hasattr(msg, "encoding") and hasattr(msg, "height"):
                import numpy as np, cv2

                enc = msg.encoding.upper()
                _dbg.warning(
                    "[col4] render: camera enc=%s h=%s w=%s data_len=%s",
                    enc,
                    getattr(msg, "height", "?"),
                    getattr(msg, "width", "?"),
                    len(msg.data) if hasattr(msg, "data") else "?",
                )
                if "32FC" in enc:
                    # Depth float32 — normalise to 0-255 and apply colormap
                    arr = np.frombuffer(msg.data, dtype=np.float32).reshape(
                        msg.height, msg.width
                    )
                    fin = arr[np.isfinite(arr)]
                    if fin.size and fin.max() > fin.min():
                        norm = (
                            ((arr - fin.min()) / (fin.max() - fin.min()) * 255)
                            .clip(0, 255)
                            .astype(np.uint8)
                        )
                    else:
                        norm = np.zeros((msg.height, msg.width), dtype=np.uint8)
                    bgr = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
                elif "16UC" in enc:
                    arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                        msg.height, msg.width
                    )
                    norm = (arr / 65535.0 * 255).astype(np.uint8)
                    bgr = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
                elif enc in ("MONO8", "8UC1"):
                    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                        msg.height, msg.width
                    )
                    bgr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
                elif enc in ("MONO16",):
                    arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                        msg.height, msg.width
                    )
                    bgr = cv2.cvtColor((arr >> 8).astype(np.uint8), cv2.COLOR_GRAY2BGR)
                elif enc in ("RGB8",):
                    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                        msg.height, msg.width, 3
                    )
                    bgr = arr[:, :, ::-1].copy()
                elif enc in ("BGR8",):
                    bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                        msg.height, msg.width, 3
                    )
                elif enc in ("RGBA8",):
                    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                        msg.height, msg.width, 4
                    )
                    bgr = arr[:, :, 2::-1].copy()
                elif enc in ("BGRA8",):
                    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                        msg.height, msg.width, 4
                    )
                    bgr = arr[:, :, :3].copy()
                else:
                    # Unknown encoding — try generic reshape
                    total = len(msg.data)
                    channels = total // (msg.height * msg.width)
                    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                        msg.height, msg.width, channels
                    )
                    bgr = (
                        arr[:, :, :3][:, :, ::-1].copy()
                        if channels >= 3
                        else cv2.cvtColor(arr[:, :, 0], cv2.COLOR_GRAY2BGR)
                    )
                ok, buf = cv2.imencode(".jpg", bgr)
                _dbg.warning(
                    "[col4] render: imencode ok=%s buf_len=%s",
                    ok,
                    len(buf) if ok else 0,
                )
                return bytes(buf) if ok else None

            # ── Tactile: gazebo_msgs/ContactsState → force heatmap ────────
            _dbg.warning("[col4] render: not a camera msg — checking tactile")
            if hasattr(msg, "states"):
                return self._render_tactile_heatmap(msgs)

        except Exception as e:
            import traceback

            _dbg.error(
                "[col4] render_sensor_image EXCEPTION: %s\n%s",
                e,
                traceback.format_exc(),
            )
        _dbg.warning("[col4] render: fell through — returning None")
        return None

    def _render_tactile_heatmap(self, msgs: list) -> bytes | None:
        """Aggregate ContactsState messages into a spatial force heatmap.

        Coordinate system:
          Contact points come from the bumper plugin in the WORLD frame
          (state.contact_positions[i].x/y).  The old code used min/max
          normalisation which spread a single-cluster press across the
          whole canvas — the top-left corner of the probe disc always
          landed at pixel (10, 10) regardless of where on the sensor the
          probe actually was.  A press at the physical sensor centre
          therefore appeared in the top-left of the UI.

          New approach:
            1. Compute the centroid (cx, cy) of all contact points.
               For a probe pressing a single spot, this IS the press
               location; for multi-point grids it's the average press.
            2. Use a fixed pixels-per-metre scale so the blob is drawn
               at canvas CENTRE relative to the centroid.  No dynamic
               stretching — a hard press and a light press render at the
               same pixel coordinates, only the colour differs.
            3. Overlay a faint crosshair + circle at the canvas centre so
               the viewer has a visual reference for "sensor centre".

          With this the common case (probe tapping centre of the sensor)
          renders as a hot blob precisely in the middle of the image.
        """
        try:
            import numpy as np, cv2

            H, W = 256, 256
            canvas = np.zeros((H, W), dtype=np.float32)

            # Collect contact points (x, y, force_magnitude) in world frame.
            points = []
            for msg in msgs:
                for state in msg.states or []:
                    f = state.total_wrench.force
                    mag = (f.x ** 2 + f.y ** 2 + f.z ** 2) ** 0.5
                    if state.contact_positions:
                        for pos in state.contact_positions:
                            points.append((pos.x, pos.y, mag))
                    else:
                        # No position data — just register "something
                        # pressed" at centroid (→ canvas centre).
                        points.append((0.0, 0.0, mag))

            if points:
                xs = np.array([p[0] for p in points], dtype=np.float64)
                ys = np.array([p[1] for p in points], dtype=np.float64)
                fs = np.array([p[2] for p in points], dtype=np.float64)

                # Centroid of the contact cloud → canvas centre.
                cx = float(np.mean(xs))
                cy = float(np.mean(ys))

                # Fixed scale: 2000 px/m  →  1 px ≈ 0.5 mm.
                # 256 px canvas therefore shows a ±64 mm view around the
                # contact centroid, plenty for any probe disc (r=8mm) and
                # for showing the sensor-sized area around it.
                PX_PER_M = 2000.0

                # Invert Y so that +Y in world (forward) points up in the
                # image — matches the observer camera's "bird's-eye" feel.
                pxs = ((xs - cx) * PX_PER_M + W / 2).astype(np.int32)
                pys = ((cy - ys) * PX_PER_M + H / 2).astype(np.int32)

                # Clip to canvas (defensive — points should already fit).
                pxs = np.clip(pxs, 0, W - 1)
                pys = np.clip(pys, 0, H - 1)

                for px, py, fv in zip(pxs, pys, fs):
                    cv2.circle(
                        canvas,
                        (int(px), int(py)),
                        radius=15,
                        color=float(fv),
                        thickness=-1,
                    )
                canvas = cv2.GaussianBlur(canvas, (31, 31), 0)
                if canvas.max() > 0:
                    canvas = (canvas / canvas.max() * 255).astype(np.uint8)
                else:
                    canvas = canvas.astype(np.uint8)
            else:
                canvas = canvas.astype(np.uint8)

            heatmap = cv2.applyColorMap(canvas, cv2.COLORMAP_JET)

            # Reference crosshair + circle at the canvas centre so the
            # user can see the "sensor centre" relative to the heat blob.
            # Drawn in BGR on the 3-channel heatmap.  Faint grey so it
            # doesn't fight with the colormap.
            cx_px, cy_px = W // 2, H // 2
            grey = (160, 160, 160)
            cv2.line(heatmap, (cx_px - 10, cy_px), (cx_px + 10, cy_px),
                     grey, 1, lineType=cv2.LINE_AA)
            cv2.line(heatmap, (cx_px, cy_px - 10), (cx_px, cy_px + 10),
                     grey, 1, lineType=cv2.LINE_AA)
            cv2.circle(heatmap, (cx_px, cy_px), 3, grey, 1,
                       lineType=cv2.LINE_AA)

            ok, buf = cv2.imencode(".jpg", heatmap)
            return bytes(buf) if ok else None
        except Exception:
            return None

    def _show_static_image(self, image_path: str) -> None:
        """Show sensor's assigned image_path photo."""
        if image_path:
            px = QPixmap(image_path)
            if not px.isNull():
                scaled = px.scaled(
                    self.lbl_capture_image.width(),
                    self.lbl_capture_image.height(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
                self.lbl_capture_image.setPixmap(scaled)
                self.lbl_capture_image.setText("")
                return
        self._clear_display()

    def _show_image_bytes(self, data: bytes | None, fallback: str) -> None:
        if data:
            img = QImage.fromData(data)
            if not img.isNull():
                px = QPixmap.fromImage(img).scaled(
                    self.lbl_capture_image.width(),
                    self.lbl_capture_image.height(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
                self.lbl_capture_image.setPixmap(px)
                self.lbl_capture_image.setText("")
                return
        self._clear_display(fallback)

    def _show_placeholder(self, text: str) -> None:
        self.lbl_capture_image.setPixmap(QPixmap())
        self.lbl_capture_image.setText(text)
        self.lbl_capture_image.setStyleSheet(
            f"color: {Colors.TEXT_MUTED}; font-size: 12px; padding: 20px;"
        )

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
    def append_separator(self, label: str = "") -> None:
        """Insert a visual divider into the log, e.g. at the start of each test."""
        self._sep_arrived.emit(label)

    @pyqtSlot(str)
    def _append_separator_main(self, label: str) -> None:
        cursor = self.log_view.textCursor()
        cursor.movePosition(QTextCursor.End)

        # Blank line before
        if self.log_view.toPlainText():
            plain_fmt = QTextCharFormat()
            plain_fmt.setForeground(QColor("#1e293b"))
            cursor.insertText("\n", plain_fmt)

        # Top rule
        rule_fmt = QTextCharFormat()
        rule_fmt.setForeground(QColor("#475569"))
        cursor.insertText("\n" + "━" * 48, rule_fmt)

        # Label line
        lbl_fmt = QTextCharFormat()
        lbl_fmt.setForeground(QColor("#e2e8f0"))
        lbl_fmt.setFontWeight(700)
        text = f"  ▶  {label}" if label else "  ▶  test"
        cursor.insertText("\n" + text, lbl_fmt)

        # Bottom rule
        cursor.insertText("\n" + "━" * 48, rule_fmt)
        cursor.insertText("\n", plain_fmt)

        self.log_view.setTextCursor(cursor)
        self.log_view.ensureCursorVisible()

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
            (self.btn_lock, Icons.LOCK()),
            (self.btn_step, Icons.STEP()),
            (self.btn_view_sensor, Icons.TARGET_SENSOR()),
            (self.btn_view_observer, Icons.OBSERVER()),
            (self.btn_clear_log, Icons.CLEAR()),
            (self.btn_copy_log, Icons.COPY()),
        ]:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(Styles.BUTTON_ICON)

        _checked_style = Styles.BUTTON_ICON + f"""
            QPushButton:checked {{
                background-color: {Colors.ACCENT_DIM};
                border: 1px solid {Colors.ACCENT};
                border-radius: 4px;
            }}
        """
        # Lock and view-toggle buttons all need the checked highlight
        for btn in (self.btn_lock, self.btn_view_sensor, self.btn_view_observer):
            btn.setStyleSheet(_checked_style)
