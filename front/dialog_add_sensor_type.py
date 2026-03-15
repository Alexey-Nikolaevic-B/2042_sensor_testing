import logging
import os

from PyQt5.QtWidgets import (
    QDialog, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QLineEdit, QPushButton, QPlainTextEdit,
    QSizePolicy, QListWidgetItem, QListWidget,
)
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QFont
from PyQt5 import uic

from ._theme import Icons, Layout, QT_DIR, LightColors as LC, LightStyles as LS

logger = logging.getLogger(__name__)


class _ParamRow(QWidget):
    remove_requested = pyqtSignal(object)

    def __init__(self, value: str = "", parent=None):
        super().__init__(parent)
        self.setFixedHeight(34)
        h = QHBoxLayout(self)
        h.setContentsMargins(0, 2, 0, 2)
        h.setSpacing(6)

        self.inp = QLineEdit()
        self.inp.setPlaceholderText("xml tag name  e.g. rzero")
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
        return {"name": self.inp.text().strip()}


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


class AddSensorTypeDialog(QDialog):
    type_saved = pyqtSignal(dict)

    def __init__(self, existing_tests: list = None, mode: str = "add",
                 prefill: dict = None, parent=None):
        super().__init__(parent)
        self._existing_tests = existing_tests or []
        self._mode           = mode
        self._prefill        = prefill or {}
        self._param_rows: list[_ParamRow] = []
        self._test_rows:  list[_TestRow]  = []
        self._drag_pos = None

        self.setWindowFlags(Qt.Dialog | Qt.FramelessWindowHint)
        self.showMaximized()

        uic.loadUi(f"{QT_DIR}/dialog_add_sensor_type.ui", self)

        self._setup_styles()
        self._populate_existing_tests()
        self._populate_detectors()
        self._connect_signals()

        # Anchor both scroll areas to top
        self.scroll_params_contents.layout().addStretch(1)
        self.scroll_tests_contents.layout().addStretch(1)

        if self._prefill:
            self._apply_prefill()

    # ── styles ────────────────────────────────────────────────────────────────

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QDialog {{ background: {LC.BG_PANEL}; }}
            QWidget {{
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
            QFrame#sep_vertical, QFrame#line_1,
            QFrame#line_2,       QFrame#line_3 {{
                color: {LC.BORDER};
            }}
            QScrollArea {{ border: none; background: transparent; }}
            QTextEdit {{
                background: {LC.BG};
                border: 1px solid {LC.BORDER};
                border-radius: 4px;
                color: {LC.TEXT};
                padding: 5px 8px;
                font-size: 12px;
            }}
            QTextEdit:focus {{ border-color: {LC.ACCENT}; }}
            {LS.SCROLLBAR}
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
        for name in ("lbl_name_section", "lbl_desc_section", "lbl_params_section",
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

        self.btn_add_param.setStyleSheet(f"""
            QPushButton {{
                background: transparent;
                color: {LC.ACCENT};
                border: 1px dashed {LC.BORDER};
                border-radius: 4px;
                padding: 5px 12px; font-size: 12px; text-align: left;
            }}
            QPushButton:hover {{ background: {LC.ACCENT_DIM}; border-color: {LC.ACCENT}; }}
        """)

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
            from src.sensors.detector import get_custom_detector_names
            for name in get_custom_detector_names():
                lw.addItem(QListWidgetItem(name))
        except Exception as e:
            logger.warning("Could not load custom detectors: %s", e)

    def _apply_prefill(self):
        d = self._prefill
        if d.get("name"):
            self.inp_name.setText(d["name"])
        if d.get("description"):
            self.inp_desc.setPlainText(d["description"])
        for p in d.get("params", []):
            self._add_param_row(value=p.get("name", ""))
        det = d.get("detection", {})
        if det.get("mode") == "custom":
            self._set_mode("custom")
            # show currently selected detector fn name
            fn = det.get("detector_fn", "")
            lbl = getattr(self, "lbl_selected_detector", None)
            if lbl and fn:
                lbl.setText(fn)
        else:
            self._set_mode("simple")
            self.inp_plugin.setText(det.get("plugin", ""))
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

    # ── keyboard / drag ───────────────────────────────────────────────────────

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Return, Qt.Key_Enter):
            self._on_save()
        else:
            super().keyPressEvent(event)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            child = self.childAt(event.pos())
            if child and child.objectName() in ("title_bar", "lbl_title"):
                self._drag_pos = event.globalPos() - self.frameGeometry().topLeft()
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._drag_pos and event.buttons() == Qt.LeftButton:
            self.move(event.globalPos() - self._drag_pos)
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        self._drag_pos = None
        super().mouseReleaseEvent(event)

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
            detection = {"mode": "simple", "plugin": self.inp_plugin.text().strip()}
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
            "description": self.inp_desc.toPlainText().strip(),
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
        import src.database.sensor_storage as db
        sensor_type = d["name"]
        db.upsert_sensor_type(
            sensor_type = sensor_type,
            description = d["description"],
            params      = d["params"],
            detection   = d["detection"],
        )
        for t in d["tests"]:
            db.upsert_type_test(
                sensor_type  = sensor_type,
                func_name    = t["func_name"],
                display_name = t["func_name"],
                description  = "",
                world_path   = "",
            )
