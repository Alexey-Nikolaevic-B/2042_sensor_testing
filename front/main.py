from PyQt5.QtWidgets import QMainWindow, QSizePolicy, QApplication
from PyQt5.QtCore import Qt, QPoint, QRect, QObject, QEvent
from PyQt5.QtGui import QPixmap, QCursor
from PyQt5 import uic

from ._theme import Colors, Styles, Icons, Layout, QT_DIR
from .widget_col_1 import ColSensors
from .widget_col_2 import ColDetails
from .widget_col_3 import ColTests
from .widget_col_4 import ColCapture
from .logic_sensor_repository import SensorRepository
from .dialog_add_sensor import AddSensorDialog

_EDGE = 6  # px from window edge that counts as resize zone

_CURSOR_MAP = {
    "tl": Qt.SizeFDiagCursor,
    "br": Qt.SizeFDiagCursor,
    "tr": Qt.SizeBDiagCursor,
    "bl": Qt.SizeBDiagCursor,
    "l": Qt.SizeHorCursor,
    "r": Qt.SizeHorCursor,
    "t": Qt.SizeVerCursor,
    "b": Qt.SizeVerCursor,
}


def _edge_at(win, global_pos) -> str | None:
    """Which edge/corner of *win* is *global_pos* in, or None."""
    pos = win.mapFromGlobal(global_pos)
    x, y, w, h = pos.x(), pos.y(), win.width(), win.height()
    on_l = x <= _EDGE
    on_r = x >= w - _EDGE
    on_t = y <= _EDGE
    on_b = y >= h - _EDGE
    if on_t and on_l:
        return "tl"
    if on_t and on_r:
        return "tr"
    if on_b and on_l:
        return "bl"
    if on_b and on_r:
        return "br"
    if on_l:
        return "l"
    if on_r:
        return "r"
    if on_t:
        return "t"
    if on_b:
        return "b"
    return None


class _ResizeEventFilter(QObject):
    """Installed on QApplication so it receives mouse events from every widget,
    including children that would otherwise swallow them."""

    def __init__(self, win):
        super().__init__(win)
        self._win = win
        self._resize_edge = None
        self._resize_start_pos = None
        self._resize_start_geom = None
        self._drag_pos = None

    def eventFilter(self, obj, event):
        win = self._win

        # Guard against the window having been deleted by Qt already
        try:
            import sip

            if sip.isdeleted(win):
                QApplication.instance().removeEventFilter(self)
                return False
        except Exception:
            pass

        # Only act on events that concern our window
        if not win.isVisible():
            return False

        t = event.type()
        if t in (QEvent.MouseMove, QEvent.MouseButtonPress, QEvent.MouseButtonRelease):
            if self._resize_edge is None:
                gp = event.globalPos()
                if not QRect(
                    win.mapToGlobal(win.rect().topLeft()), win.size()
                ).contains(gp):
                    return False

        # ── mouse move (no button) → update cursor ────────────────────────────
        if event.type() == QEvent.MouseMove and not (event.buttons() & Qt.LeftButton):
            edge = _edge_at(win, event.globalPos())
            win.setCursor(_CURSOR_MAP.get(edge, Qt.ArrowCursor))

        # ── press ─────────────────────────────────────────────────────────────
        elif (
            event.type() == QEvent.MouseButtonPress and event.button() == Qt.LeftButton
        ):
            edge = _edge_at(win, event.globalPos())
            if edge:
                self._resize_edge = edge
                self._resize_start_pos = event.globalPos()
                self._resize_start_geom = win.geometry()
                return True  # consume — prevent child widgets acting on it
            # title bar drag
            tb = win.wt_titlebar
            tb_rect = QRect(win.mapToGlobal(tb.pos()), tb.size())
            if tb_rect.contains(event.globalPos()):
                self._drag_pos = event.globalPos() - win.frameGeometry().topLeft()

        # ── move (button held) → drag or resize ───────────────────────────────
        elif event.type() == QEvent.MouseMove and (event.buttons() & Qt.LeftButton):
            if self._resize_edge:
                self._do_resize(event.globalPos())
                return True
            if self._drag_pos is not None:
                win.move(event.globalPos() - self._drag_pos)
                return True

        # ── release ───────────────────────────────────────────────────────────
        elif (
            event.type() == QEvent.MouseButtonRelease
            and event.button() == Qt.LeftButton
        ):
            self._resize_edge = None
            self._resize_start_pos = None
            self._resize_start_geom = None
            self._drag_pos = None
            win.setCursor(Qt.ArrowCursor)

        return False  # never consume — buttons must still receive their events

    def _do_resize(self, global_pos):
        delta = global_pos - self._resize_start_pos
        dx, dy = delta.x(), delta.y()
        g = self._resize_start_geom
        x, y, w, h = g.x(), g.y(), g.width(), g.height()
        min_w = self._win.minimumWidth() or 400
        min_h = self._win.minimumHeight() or 300
        edge = self._resize_edge

        if "r" in edge:
            w = max(min_w, w + dx)
        if "b" in edge:
            h = max(min_h, h + dy)
        if "l" in edge:
            new_w = max(min_w, w - dx)
            x += w - new_w
            w = new_w
        if "t" in edge:
            new_h = max(min_h, h - dy)
            y += h - new_h
            h = new_h

        self._win.move(x, y)
        self._win.resize(w, h)


class Main_UI(QMainWindow):

    def __init__(self):
        super().__init__()
        self._is_maximized = False
        self._drag_pos = None
        self._resize_edge = None  # active edge/corner being dragged
        self._resize_start_geom = None  # window geometry at drag start
        self._resize_start_pos = None  # global cursor pos at drag start
        self._EDGE = 6  # px from edge that counts as resize zone

        self._init_ui()
        self._build_columns()
        self._setup_styles()
        self._connect_signals()
        self._load_all_sensors()

        # Install on QApplication so mouse events are caught before any child
        # widget consumes them — necessary for frameless window resize to work.
        self._resize_filter = _ResizeEventFilter(self)
        QApplication.instance().installEventFilter(self._resize_filter)

    def closeEvent(self, event):
        QApplication.instance().removeEventFilter(self._resize_filter)
        super().closeEvent(event)

    def _init_ui(self):
        uic.loadUi(f"{QT_DIR}/main.ui", self)
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_TranslucentBackground)

    def _build_columns(self):
        self.col_1 = ColSensors()
        self.col_2 = ColDetails()
        self.col_3 = ColTests()
        self.col_4 = ColCapture()

        self.test_page = self.col_3

        for col in (self.col_1, self.col_2, self.col_3, self.col_4):
            col.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.splitter_main.addWidget(col)

        self.splitter_main.setSizes([300, 300, 300, 300])

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QMainWindow, QWidget#centralwidget {{
                background-color: {Colors.BG_APP};
                border-radius: 8px;
            }}
            QWidget#wt_titlebar {{
                background-color: {Colors.BG_TITLEBAR};
                border-bottom: 1px solid {Colors.BORDER};
            }}
            QLabel#lbl_app_title {{
                color: {Colors.TEXT_SECONDARY};
                font-size: 12px;
                font-weight: bold;
            }}
            QSplitter::handle {{
                background-color: {Colors.SPLITTER};
                width: 1px;
            }}
        """)
        self.app_icon.setPixmap(Icons.SENSOR().pixmap(Layout.ICON_SIZE_MD))
        for btn, icon in [
            (self.btn_minimize, Icons.MINIMIZE()),
            (self.btn_maximize, Icons.MAXIMIZE()),
            (self.btn_close, Icons.CLOSE()),
        ]:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(Styles.BUTTON_ICON)

    def _connect_signals(self):
        self.btn_minimize.clicked.connect(self.showMinimized)
        self.btn_maximize.clicked.connect(self._toggle_maximize)
        self.btn_close.clicked.connect(self.close)

        self.col_1.sensor_selected.connect(self._on_sensor_selected)
        self.col_1.add_requested.connect(self._on_add_sensor)
        self.col_1.delete_requested.connect(self._on_delete_sensor)
        self.col_1.type_updated.connect(self._on_type_updated)

        self.col_2.sensor_updated.connect(self._on_sensor_edited)

        self.col_3._runner_log_forward = self.col_4.append_log
        self.col_3.test_result_ready.connect(self.col_4.load_test_result)

    def _load_all_sensors(self):
        repo = SensorRepository.instance()
        sensors = repo.all_sensors()
        types = repo.get_types()
        self.col_1.load_sensors(sensors, types)

        repo.sensor_added.connect(lambda _: self._reload_sensors())
        repo.sensor_updated.connect(lambda _: self._reload_sensors())
        repo.test_updated.connect(self._on_test_updated)

    def _on_type_updated(self, sensor_type: str) -> None:
        """Refresh col_3 when a type's tests change.
        Must reload from DB first — repo holds stale in-memory data."""
        from .logic_sensor_repository import SensorRepository

        repo = SensorRepository.instance()
        sensor_id = self.col_1.selected_sensor_id()
        if not sensor_id:
            return
        # Force repo to re-read this sensor's tests from DB
        repo._reload_sensor_from_db(sensor_id)
        sensor = repo.get_sensor(sensor_id)
        if sensor and sensor.get("type") == sensor_type:
            self.col_3.load_sensor(sensor)

    def _on_sensor_selected(self, sensor_id: str):
        repo = SensorRepository.instance()
        sensor = repo.get_sensor(sensor_id)
        if not sensor:
            return
        data = sensor
        self.col_2.load_sensor(data)
        self.col_3.load_sensor(data)

    def _reload_sensors(self):
        repo = SensorRepository.instance()
        prev_id = self.col_1._selected_id
        self.col_1.load_sensors(repo.all_sensors(), repo.get_types())
        if prev_id:
            self.col_1.set_selected(prev_id)

            sensor = repo.get_sensor(prev_id)
            if sensor:
                self.col_2.load_sensor(sensor)
                self.col_3.load_sensor(sensor)

    def _on_test_updated(self, sensor_id: str):
        repo = SensorRepository.instance()
        sensor = repo.get_sensor(sensor_id)
        if sensor:
            self.col_1.refresh_sensor(sensor)

    def set_core(self, core) -> None:
        self._core = core

    def _on_add_sensor(self):
        core = getattr(self, "_core", None)
        dlg = AddSensorDialog(parent=self, core=core)
        dlg.sensor_saved.connect(self._on_sensor_saved)
        dlg.exec_()

    def _on_sensor_edited(self, sensor_dict: dict):
        repo = SensorRepository.instance()
        sensor_id = sensor_dict.get("id")
        if not sensor_id:
            return
        try:
            updated = repo.update_sensor(sensor_id, sensor_dict)
            self.col_1.refresh_sensor(updated)
        except Exception as e:
            from PyQt5.QtWidgets import QMessageBox

            QMessageBox.warning(self, "Update failed", str(e))

    def _on_delete_sensor(self, sensor_id: str):
        repo = SensorRepository.instance()
        try:
            was_selected = self.col_1._selected_id == sensor_id
            repo.delete_sensor(sensor_id)
            self.col_1.remove_cell(sensor_id)
            if was_selected:
                self.col_2.clear()
                self.col_3.clear()
        except Exception as e:
            from PyQt5.QtWidgets import QMessageBox

            QMessageBox.warning(self, "Delete failed", str(e))

    def _on_sensor_saved(self, sensor_dict: dict):
        repo = SensorRepository.instance()
        try:
            repo.add_sensor(sensor_dict)
            # repo.sensor_added signal triggers _reload_sensors automatically
        except Exception as e:
            from PyQt5.QtWidgets import QMessageBox

            QMessageBox.warning(self, "Save failed", str(e))

    # ── resize / drag ─────────────────────────────────────────────────────────
    # Handled entirely by _ResizeEventFilter (installed on QApplication).
    # These stubs exist only so the filter can call them cleanly.

    def _toggle_maximize(self):
        if self._is_maximized:
            self.showNormal()
        else:
            self.showMaximized()
        self._is_maximized = not self._is_maximized
