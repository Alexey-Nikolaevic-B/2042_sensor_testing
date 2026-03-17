import logging
import os

from PyQt5.QtCore import pyqtSignal, Qt, QObject, QEvent, QRect
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QDialog, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QLineEdit, QPushButton, QPlainTextEdit,
    QSizePolicy, QListWidgetItem, QListWidget, QApplication,
)
from PyQt5 import uic

from ._theme import Icons, Layout, QT_DIR, LightColors as LC, LightStyles as LS

logger = logging.getLogger(__name__)


class _ParamRow(QWidget):
    """Single field: accepts "width", "camera/width", or "camera/lens/width"."""
    remove_requested = pyqtSignal(object)

    def __init__(self, value: str = "", parent=None):
        super().__init__(parent)
        self.setFixedHeight(34)
        h = QHBoxLayout(self)
        h.setContentsMargins(0, 2, 0, 2)
        h.setSpacing(6)

        self.inp = QLineEdit()
        self.inp.setPlaceholderText("e.g. width  or  camera/width  or  camera/lens/width")
        self.inp.setStyleSheet(LS.INPUT)
        if value:
            self.inp.setText(value)

        btn = QPushButton()
        btn.setFixedSize(24, 24)
        btn.setIcon(Icons.CLEAR())
        btn.setIconSize(Layout.ICON_SIZE_SM)
        btn.setStyleSheet(LS.BUTTON_ICON)
        btn.clicked.connect(lambda: self.remove_requested.emit(self))

        h.addWidget(self.inp)
        h.addWidget(btn)

    def data(self) -> dict:
        text = self.inp.text().strip()
        if "/" in text:
            parts = text.rsplit("/", 1)
            return {"path": parts[0], "name": parts[1]}
        return {"name": text}


class _TestRow(QWidget):
    remove_requested = pyqtSignal(object)

    def __init__(self, func_name: str, parent=None):
        super().__init__(parent)
        self._func_name = func_name
        self.setFixedHeight(36)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setStyleSheet(f"""
            QWidget {{
                background: {LC.BG};
                border-bottom: 1px solid {LC.BG_HOVER};
            }}
        """)
        h = QHBoxLayout(self)
        h.setContentsMargins(10, 0, 6, 0)
        h.setSpacing(8)

        lbl = QLabel(func_name)
        lbl.setStyleSheet(
            f"color: {LC.TEXT}; font-size: 12px; background: transparent; border: none;"
        )
        btn_rm = QPushButton()
        btn_rm.setFixedSize(24, 24)
        btn_rm.setIcon(Icons.CLEAR())
        btn_rm.setIconSize(Layout.ICON_SIZE_SM)
        btn_rm.setStyleSheet(LS.BUTTON_ICON)
        btn_rm.clicked.connect(lambda: self.remove_requested.emit(self))

        h.addWidget(lbl, stretch=1)
        h.addWidget(btn_rm)

    def data(self) -> dict:
        return {"func_name": self._func_name}


def _lay_insert(layout, widget):
    """Insert widget before the trailing stretch (if any), preserving top-align."""
    count = layout.count()
    if count and layout.itemAt(count - 1).spacerItem():
        layout.insertWidget(count - 1, widget)
    else:
        layout.addWidget(widget)


def _lay_remove(layout, widget):
    """Remove widget; keep trailing stretch intact."""
    layout.removeWidget(widget)


class _PluginRow(QWidget):
    """One plugin filename entry in the detection section."""
    remove_requested = pyqtSignal(object)

    def __init__(self, value: str = "", parent=None):
        super().__init__(parent)
        self.setFixedHeight(34)
        h = QHBoxLayout(self)
        h.setContentsMargins(0, 2, 0, 2)
        h.setSpacing(6)

        self.inp = QLineEdit()
        self.inp.setPlaceholderText("e.g. libgazebo_ros_camera.so")
        self.inp.setStyleSheet(LS.INPUT)
        if value:
            self.inp.setText(value)

        btn = QPushButton()
        btn.setFixedSize(24, 24)
        btn.setIcon(Icons.CLEAR())
        btn.setIconSize(Layout.ICON_SIZE_SM)
        btn.setStyleSheet(LS.BUTTON_ICON)
        btn.clicked.connect(lambda: self.remove_requested.emit(self))

        h.addWidget(self.inp)
        h.addWidget(btn)

    def value(self) -> str:
        return self.inp.text().strip()


_EDGE = 6
_CURSOR_MAP = {
    "tl": Qt.SizeFDiagCursor, "br": Qt.SizeFDiagCursor,
    "tr": Qt.SizeBDiagCursor, "bl": Qt.SizeBDiagCursor,
    "l":  Qt.SizeHorCursor,   "r":  Qt.SizeHorCursor,
    "t":  Qt.SizeVerCursor,   "b":  Qt.SizeVerCursor,
}


def _edge_at(win, global_pos):
    pos = win.mapFromGlobal(global_pos)
    x, y, w, h = pos.x(), pos.y(), win.width(), win.height()
    on_l = x <= _EDGE;  on_r = x >= w - _EDGE
    on_t = y <= _EDGE;  on_b = y >= h - _EDGE
    if on_t and on_l: return "tl"
    if on_t and on_r: return "tr"
    if on_b and on_l: return "bl"
    if on_b and on_r: return "br"
    if on_l: return "l"
    if on_r: return "r"
    if on_t: return "t"
    if on_b: return "b"
    return None


class _WinFilter(QObject):
    """App-level event filter: handles both title-bar drag and edge resize
    for a single frameless window/dialog."""

    def __init__(self, win, title_bar_attr="title_bar", resizable=True):
        super().__init__(win)
        self._win            = win
        self._tb_attr        = title_bar_attr
        self._resizable      = resizable
        self._drag_pos       = None
        self._resize_edge    = None
        self._resize_start_p = None
        self._resize_start_g = None

    def eventFilter(self, obj, event):
        win = self._win
        try:
            import sip
            if sip.isdeleted(win):
                QApplication.instance().removeEventFilter(self)
                return False
        except Exception:
            pass

        if not win.isVisible():
            return False

        t = event.type()

        # ── guard: ignore events outside our window unless mid-drag/resize ──
        if t in (QEvent.MouseMove, QEvent.MouseButtonPress,
                 QEvent.MouseButtonRelease):
            if self._drag_pos is None and self._resize_edge is None:
                gp = event.globalPos()
                if not QRect(win.mapToGlobal(win.rect().topLeft()),
                             win.size()).contains(gp):
                    return False

        # ── cursor shape (no button) ─────────────────────────────────────────
        if t == QEvent.MouseMove and not (event.buttons() & Qt.LeftButton):
            if self._resizable:
                edge = _edge_at(win, event.globalPos())
                win.setCursor(_CURSOR_MAP.get(edge, Qt.ArrowCursor))
            return False

        # ── press ────────────────────────────────────────────────────────────
        elif t == QEvent.MouseButtonPress and event.button() == Qt.LeftButton:
            if self._resizable:
                edge = _edge_at(win, event.globalPos())
                if edge:
                    self._resize_edge    = edge
                    self._resize_start_p = event.globalPos()
                    self._resize_start_g = win.geometry()
                    return True
            # title bar drag
            tb = getattr(win, self._tb_attr, None)
            if tb:
                tb_rect = QRect(win.mapToGlobal(tb.pos()), tb.size())
                if tb_rect.contains(event.globalPos()):
                    self._drag_pos = (event.globalPos()
                                      - win.frameGeometry().topLeft())

        # ── move ─────────────────────────────────────────────────────────────
        elif t == QEvent.MouseMove and (event.buttons() & Qt.LeftButton):
            if self._resize_edge:
                self._do_resize(event.globalPos())
                return True
            if self._drag_pos is not None:
                win.move(event.globalPos() - self._drag_pos)
                return True

        # ── release ──────────────────────────────────────────────────────────
        elif t == QEvent.MouseButtonRelease and event.button() == Qt.LeftButton:
            self._resize_edge    = None
            self._resize_start_p = None
            self._resize_start_g = None
            self._drag_pos       = None
            win.setCursor(Qt.ArrowCursor)

        return False

    def _do_resize(self, global_pos):
        delta  = global_pos - self._resize_start_p
        dx, dy = delta.x(), delta.y()
        g      = self._resize_start_g
        x, y, w, h = g.x(), g.y(), g.width(), g.height()
        min_w  = self._win.minimumWidth()  or 400
        min_h  = self._win.minimumHeight() or 300
        edge   = self._resize_edge
        if "r" in edge: w = max(min_w, w + dx)
        if "b" in edge: h = max(min_h, h + dy)
        if "l" in edge:
            new_w = max(min_w, w - dx); x += w - new_w; w = new_w
        if "t" in edge:
            new_h = max(min_h, h - dy); y += h - new_h; h = new_h
        self._win.move(x, y)
        self._win.resize(w, h)


class AddSensorTypeDialog(QDialog):
    type_saved = pyqtSignal(dict)

    def __init__(self, existing_tests: list = None, mode: str = "add",
                 prefill: dict = None, parent=None):
        super().__init__(parent)
        self._existing_tests = existing_tests or []
        self._mode           = mode
        self._prefill        = prefill or {}
        self._param_rows:  list[_ParamRow]  = []
        self._plugin_rows: list[_PluginRow] = []
        self._test_rows:   list[_TestRow]   = []

        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setMinimumSize(920, 680)
        self.resize(960, 700)

        uic.loadUi(f"{QT_DIR}/dialog_add_sensor_type.ui", self)

        self._setup_styles()
        self._populate_existing_tests()
        self._populate_detectors()
        self._connect_signals()

        self.scroll_params_contents.layout().addStretch(1)
        self.scroll_plugins_contents.layout().addStretch(1)
        self.scroll_tests_contents.layout().addStretch(1)

        if self._prefill:
            self._apply_prefill()

        self._win_filter = _WinFilter(self, title_bar_attr="title_bar", resizable=True)
        QApplication.instance().installEventFilter(self._win_filter)

    def closeEvent(self, event):
        QApplication.instance().removeEventFilter(self._win_filter)
        super().closeEvent(event)

    # ── styles ────────────────────────────────────────────────────────────────

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QDialog#AddSensorTypeDialog {{
                background: {LC.BG_PANEL};
            }}

            /* ── panels ── */
            QWidget#panel_left,
            QWidget#widget_simple,
            QWidget#widget_custom {{
                background: {LC.BG_PANEL};
                color: {LC.TEXT};
                font-size: 13px;
            }}
            QWidget#title_bar {{
                background: {LC.BG};
                border-bottom: 1px solid {LC.BORDER};
            }}
            QWidget#panel_right {{
                background: {LC.BG};
                border-left: 1px solid {LC.BORDER};
            }}
            QWidget#panel_footer {{
                background: {LC.BG};
                border-top: 1px solid {LC.BORDER};
            }}

            /* ── scroll areas & their viewports ── */
            QScrollArea#scroll_params,
            QScrollArea#scroll_tests,
            QScrollArea#scroll_plugins {{
                border: 1px solid {LC.BORDER};
                border-radius: 4px;
                background: {LC.BG};
            }}
            QWidget#scroll_params_contents,
            QWidget#scroll_tests_contents,
            QWidget#scroll_plugins_contents {{
                background: {LC.BG};
            }}

            /* ── list widgets ── */
            QListWidget#list_existing_tests,
            QListWidget#list_existing_detectors {{
                border: 1px solid {LC.BORDER};
                background: {LC.BG};
                color: {LC.TEXT};
                outline: none;
            }}
            QListWidget#list_existing_tests::item,
            QListWidget#list_existing_detectors::item {{
                border-bottom: 1px solid {LC.BG_HOVER};
                color: {LC.TEXT};
                padding: 4px;
            }}
            QListWidget#list_existing_tests::item:hover,
            QListWidget#list_existing_detectors::item:hover {{
                background: {LC.BG_HOVER};
            }}
            QListWidget#list_existing_tests::item:selected,
            QListWidget#list_existing_detectors::item:selected {{
                background: {LC.ACCENT_DIM};
                color: {LC.TEXT};
            }}

            /* ── divider lines ── */
            QFrame#sep_vertical {{
                background: {LC.BORDER};
                border: none;
                max-width: 1px;
            }}
            QFrame#line_2, QFrame#line_3 {{
                background: {LC.BORDER};
                border: none;
                max-height: 1px;
            }}

            /* ── labels ── */
            QLabel {{
                background: transparent;
                color: {LC.TEXT};
                font-size: 13px;
            }}

            /* ── inputs ── */
            QLineEdit {{
                background: {LC.BG_INPUT};
                border: 1px solid {LC.BORDER};
                border-radius: 3px;
                color: {LC.TEXT};
                padding: 4px 8px;
                font-size: 13px;
            }}
            QLineEdit:focus {{ border-color: {LC.ACCENT}; }}

            QTextEdit {{
                background: {LC.BG};
                border: 1px solid {LC.BORDER};
                border-radius: 4px;
                color: {LC.TEXT};
                padding: 5px 8px;
                font-size: 12px;
            }}
            QTextEdit:focus {{ border-color: {LC.ACCENT}; }}

            /* ── scrollbars ── */
            QScrollBar:vertical {{
                background: {LC.BG_HOVER}; width: 5px; margin: 0;
            }}
            QScrollBar::handle:vertical {{
                background: {LC.BORDER}; border-radius: 2px; min-height: 20px;
            }}
            QScrollBar::handle:vertical:hover {{ background: #aaaaaa; }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical
                {{ height: 0; background: transparent; }}
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical
                {{ background: transparent; }}
            QScrollBar:horizontal {{ height: 0; background: transparent; }}
        """)

        self.lbl_title.setStyleSheet(
            f"background: transparent; color: {LC.TEXT}; font-size: 15px; font-weight: 700;"
        )

        self.btn_close.setIcon(Icons.CLOSE())
        self.btn_close.setIconSize(Layout.ICON_SIZE_SM)
        self.btn_close.setStyleSheet(LS.BUTTON_ICON)

        _sec = (
            f"color: {LC.TEXT_SEC}; font-size: 11px; font-weight: 600;"
            " letter-spacing: 0.5px; background: transparent;"
        )
        for name in ("lbl_name_section", "lbl_params_section",
                     "lbl_detect_section", "lbl_tests_section", "lbl_added_tests"):
            w = getattr(self, name, None)
            if w:
                w.setStyleSheet(_sec)

        self.lbl_status.setStyleSheet(
            f"color: {LC.ERROR}; font-size: 12px; background: transparent;"
        )

        # File path hints
        tests_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "src", "tests", "_common.py")
        )
        lbl_f = getattr(self, "lbl_tests_file", None)
        if lbl_f:
            lbl_f.setText(f"Register tests in:  {tests_path}")
            lbl_f.setStyleSheet(
                f"color: {LC.TEXT_MUTED}; font-size: 10px; background: transparent;"
            )

        det_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "src", "sensors", "detector.py")
        )
        lbl_d = getattr(self, "lbl_detector_file", None)
        if lbl_d:
            lbl_d.setText(f"Add detectors in:  {det_path}")
            lbl_d.setStyleSheet(
                f"color: {LC.TEXT_MUTED}; font-size: 10px; background: transparent;"
            )

        for name in ("lbl_tests_hint", "lbl_params_hint", "lbl_detect_hint",
                     "lbl_plugin_hint", "lbl_custom_hint"):
            w = getattr(self, name, None)
            if w:
                w.setStyleSheet(
                    f"color: {LC.TEXT_SEC}; font-size: 11px; background: transparent;"
                )

        for name in ("inp_name", "inp_plugin", "inp_test_search", "inp_detector_search"):
            w = getattr(self, name, None)
            if w:
                w.setStyleSheet(LS.INPUT)

        self.list_existing_tests.setStyleSheet(LS.LIST_WIDGET)
        lw_det = getattr(self, "list_existing_detectors", None)
        if lw_det:
            lw_det.setStyleSheet(LS.LIST_WIDGET)

        _scroll_style = f"background: {LC.BG}; border: 1px solid {LC.BORDER}; border-radius: 4px;"
        _vp_style     = f"background: {LC.BG};"
        for sa_name in ("scroll_params", "scroll_tests", "scroll_plugins"):
            sa = getattr(self, sa_name, None)
            if sa:
                sa.setStyleSheet(_scroll_style)
                sa.viewport().setStyleSheet(_vp_style)

        for btn_id in ("btn_mode_simple", "btn_mode_custom"):
            btn = getattr(self, btn_id)
            btn.setStyleSheet(f"""
                QPushButton {{
                    border: 1px solid {LC.BORDER};
                    padding: 5px 14px; font-size: 12px; font-weight: 600;
                    background: {LC.BG}; color: {LC.TEXT_SEC};
                }}
                QPushButton:checked {{
                    background: {LC.ACCENT_DIM}; color: {LC.ACCENT};
                    border-color: {LC.ACCENT};
                }}
                QPushButton:hover:!checked {{ background: {LC.BG_HOVER}; color: {LC.TEXT}; }}
            """)
        self.btn_mode_simple.setStyleSheet(
            self.btn_mode_simple.styleSheet() +
            "QPushButton { border-top-left-radius:4px; border-bottom-left-radius:4px; border-right:none; }"
        )
        self.btn_mode_custom.setStyleSheet(
            self.btn_mode_custom.styleSheet() +
            "QPushButton { border-top-right-radius:4px; border-bottom-right-radius:4px; }"
        )

        self.btn_save.setStyleSheet(LS.BUTTON_PRIMARY)
        self.btn_cancel.setStyleSheet(LS.BUTTON_CANCEL)

        _dashed_btn_style = f"""
            QPushButton {{
                background: transparent;
                color: {LC.ACCENT};
                border: 1px dashed {LC.BORDER};
                border-radius: 4px;
                padding: 5px 12px; font-size: 12px; text-align: left;
            }}
            QPushButton:hover {{ background: {LC.ACCENT_DIM}; border-color: {LC.ACCENT}; }}
        """
        self.btn_add_param.setStyleSheet(_dashed_btn_style)
        self.btn_add_plugin.setStyleSheet(_dashed_btn_style)

        for btn in (self.btn_add_selected_tests,):
            btn.setStyleSheet(LS.BUTTON_DEFAULT)

        btn_add_det = getattr(self, "btn_add_selected_detector", None)
        if btn_add_det:
            btn_add_det.setStyleSheet(LS.BUTTON_DEFAULT)

        if self._mode == "edit":
            self.lbl_title.setText("Edit sensor type")
            self.btn_save.setText("Save changes")

    # ── populate ──────────────────────────────────────────────────────────────

    def _populate_existing_tests(self):
        for name in self._existing_tests:
            self.list_existing_tests.addItem(QListWidgetItem(name))

    def _populate_detectors(self):
        lw = getattr(self, "list_existing_detectors", None)
        if lw is None:
            return
        lw.clear()
        try:
            from detector import get_custom_detector_names
            for name in get_custom_detector_names():
                lw.addItem(QListWidgetItem(name))
        except Exception as e:
            logger.warning("Could not load custom detectors: %s", e)

    def _apply_prefill(self):
        d = self._prefill
        if d.get("name"):
            self.inp_name.setText(d["name"])
        for p in d.get("params", []):
            if isinstance(p, dict):
                path_str = p.get("path", "")
                name_str = p.get("name", "")
                self._add_param_row(f"{path_str}/{name_str}" if path_str else name_str)
            else:
                self._add_param_row(str(p))
        det = d.get("detection", {})
        if det.get("mode") == "custom":
            self._set_mode("custom")
            fn = det.get("detector_fn", "")
            lbl = getattr(self, "lbl_selected_detector", None)
            if lbl and fn:
                lbl.setText(fn)
        else:
            self._set_mode("simple")
            plugins = det.get("plugins") or (
                [det["plugin"]] if det.get("plugin") else []
            )
            for p in plugins:
                self._add_plugin_row(p)
        for t in d.get("tests", []):
            self._add_test_row(t.get("func_name", ""))

    # ── signals ───────────────────────────────────────────────────────────────

    def _connect_signals(self):
        self.btn_close.clicked.connect(self.reject)
        self.btn_cancel.clicked.connect(self.reject)
        self.btn_save.clicked.connect(self._on_save)

        self.btn_mode_simple.clicked.connect(lambda: self._set_mode("simple"))
        self.btn_mode_custom.clicked.connect(lambda: self._set_mode("custom"))

        self.btn_add_param.clicked.connect(self._add_param_row)
        self.btn_add_plugin.clicked.connect(self._add_plugin_row)

        self.inp_test_search.textChanged.connect(self._filter_tests)
        self.btn_add_selected_tests.clicked.connect(self._add_selected_tests)
        self.list_existing_tests.itemDoubleClicked.connect(
            lambda item: self._add_test_row(item.text())
        )

        btn_add_det = getattr(self, "btn_add_selected_detector", None)
        if btn_add_det:
            btn_add_det.clicked.connect(self._select_detector)

        lw_det = getattr(self, "list_existing_detectors", None)
        if lw_det:
            lw_det.itemDoubleClicked.connect(
                lambda item: self._set_selected_detector(item.text())
            )

        inp_det_search = getattr(self, "inp_detector_search", None)
        if inp_det_search:
            inp_det_search.textChanged.connect(self._filter_detectors)

    # ── keyboard ──────────────────────────────────────────────────────────────

    def showEvent(self, event):
        super().showEvent(event)
        # Re-apply after show so Qt resolves styles against this widget,
        # not the parent chain. Without this, styles break when not maximized.
        self._setup_styles()

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Return, Qt.Key_Enter):
            self._on_save()
        else:
            super().keyPressEvent(event)

    # ── detection mode ────────────────────────────────────────────────────────

    def _set_mode(self, mode: str):
        simple = (mode == "simple")
        self.btn_mode_simple.setChecked(simple)
        self.btn_mode_custom.setChecked(not simple)
        self.widget_simple.setVisible(simple)
        self.widget_custom.setVisible(not simple)

    def _filter_detectors(self, text: str):
        lw = getattr(self, "list_existing_detectors", None)
        if not lw:
            return
        text = text.strip().lower()
        for i in range(lw.count()):
            item = lw.item(i)
            item.setHidden(bool(text) and text not in item.text().lower())

    def _select_detector(self):
        lw = getattr(self, "list_existing_detectors", None)
        if not lw:
            return
        sel = lw.selectedItems()
        if sel:
            self._set_selected_detector(sel[0].text())

    def _set_selected_detector(self, name: str):
        lbl = getattr(self, "lbl_selected_detector", None)
        if lbl:
            lbl.setText(name)
            lbl.setStyleSheet(
                f"color: {LC.ACCENT}; font-size: 12px; font-weight: 600;"
                " background: transparent;"
            )

    # ── param rows ────────────────────────────────────────────────────────────

    def _add_param_row(self, value: str = ""):
        row = _ParamRow(value=value, parent=self)
        row.remove_requested.connect(self._remove_param_row)
        self._param_rows.append(row)
        _lay_insert(self.scroll_params_contents.layout(), row)

    def _remove_param_row(self, row: _ParamRow):
        self._param_rows.remove(row)
        _lay_remove(self.scroll_params_contents.layout(), row)
        row.deleteLater()

    # ── plugin rows ───────────────────────────────────────────────────────────

    def _add_plugin_row(self, value: str = ""):
        row = _PluginRow(value=value, parent=self)
        row.remove_requested.connect(self._remove_plugin_row)
        self._plugin_rows.append(row)
        _lay_insert(self.scroll_plugins_contents.layout(), row)

    def _remove_plugin_row(self, row: _PluginRow):
        self._plugin_rows.remove(row)
        _lay_remove(self.scroll_plugins_contents.layout(), row)
        row.deleteLater()

    # ── test rows ─────────────────────────────────────────────────────────────

    def _filter_tests(self, text: str):
        text = text.strip().lower()
        for i in range(self.list_existing_tests.count()):
            item = self.list_existing_tests.item(i)
            item.setHidden(bool(text) and text not in item.text().lower())

    def _add_selected_tests(self):
        for item in self.list_existing_tests.selectedItems():
            self._add_test_row(item.text())

    def _add_test_row(self, func_name: str):
        if any(r.data()["func_name"] == func_name for r in self._test_rows):
            return
        row = _TestRow(func_name, parent=self)
        row.remove_requested.connect(self._remove_test_row)
        self._test_rows.append(row)
        _lay_insert(self.scroll_tests_contents.layout(), row)

    def _remove_test_row(self, row: _TestRow):
        self._test_rows.remove(row)
        _lay_remove(self.scroll_tests_contents.layout(), row)
        row.deleteLater()

    # ── save ──────────────────────────────────────────────────────────────────

    def _on_save(self):
        self.lbl_status.setText("")
        name = self.inp_name.text().strip()
        if not name:
            self.lbl_status.setText("Sensor type name is required.")
            return

        if self.btn_mode_simple.isChecked():
            plugins = [r.value() for r in self._plugin_rows if r.value()]
            detection = {"mode": "simple", "plugins": plugins}
        else:
            fn_name = ""
            lbl = getattr(self, "lbl_selected_detector", None)
            if lbl:
                fn_name = lbl.text().strip()
            if not fn_name:
                self.lbl_status.setText("Select a detector function.")
                return
            detection = {"mode": "custom", "detector_fn": fn_name}

        definition = {
            "name":        name,
            "params":      [r.data() for r in self._param_rows if r.data()["name"]],
            "detection":   detection,
            "tests":       [r.data() for r in self._test_rows],
        }

        try:
            self._persist(definition)
        except Exception as exc:
            logger.error("AddSensorTypeDialog save failed: %s", exc, exc_info=True)
            self.lbl_status.setText(f"Save failed: {exc}")
            return

        self.type_saved.emit(definition)
        self.accept()

    def _persist(self, d: dict):
        import src.sensor_storage as db

        new_name = d["name"]
        old_name = self._prefill.get("name", new_name) if self._prefill else new_name

        # Rename if type name changed
        if old_name and old_name != new_name:
            db.rename_sensor_type(old_name, new_name)

        # Upsert type definition
        db.upsert_sensor_type(
            sensor_type = new_name,
            description = "",
            params      = d["params"],
            detection   = d["detection"],
        )

        # Delete tests removed from the dialog, upsert kept/new ones
        new_func_names = {t["func_name"] for t in d["tests"]}
        existing = {t["func_name"] for t in db.get_type_tests(new_name)}

        for removed in existing - new_func_names:
            db.delete_type_test(new_name, removed)

        added = new_func_names - existing
        for t in d["tests"]:
            db.upsert_type_test(
                sensor_type  = new_name,
                func_name    = t["func_name"],
                display_name = t["func_name"],
                description  = "",
                world_path   = "",
            )
            # Push newly added tests down to all existing sensors of this type
            if t["func_name"] in added:
                db.sync_type_tests_to_sensor_by_type(new_name, t["func_name"])