from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFrame, QLabel,
    QSizePolicy, QMenu, QAction, QPushButton,
)
from PyQt5.QtCore import pyqtSignal, Qt, QObject, QEvent
from PyQt5.QtGui import QIcon
from PyQt5 import uic

from ._theme import Colors, Styles, Icons, Layout, QT_DIR, ICON_DIR, LightColors as LC, LightStyles as LS


def _install_drag_hint(toolbar: QWidget, side: str = "right", thickness: int = 2) -> None:
    from PyQt5.QtWidgets import QFrame as _QFrame

    normal_color = Colors.SPLITTER
    hover_color  = Colors.ACCENT

    class _Strip(_QFrame):
        def _reposition(self):
            h = self.parent().height()
            if side == "right":
                self.setGeometry(self.parent().width() - thickness, 0, thickness, h)
            else:
                self.setGeometry(0, 0, thickness, h)

        def enterEvent(self, _event):
            self.setStyleSheet(f"background: {hover_color}; border: none;")

        def leaveEvent(self, _event):
            self.setStyleSheet(f"background: {normal_color}; border: none;")

    strip = _Strip(toolbar)
    strip.setStyleSheet(f"background: {normal_color}; border: none;")
    strip.setCursor(Qt.SizeHorCursor)
    strip.raise_()
    strip._reposition()

    class _ResizeFilter(QObject):
        def eventFilter(self, obj, event):
            if event.type() == QEvent.Resize:
                strip._reposition()
            return False

    _f = _ResizeFilter(toolbar)
    toolbar.installEventFilter(_f)


# ── Sensor type cell ──────────────────────────────────────────────────────────

class SensorTypeCell(QFrame):
    clicked        = pyqtSignal(str)   # sensor_type
    delete_clicked = pyqtSignal(str)   # sensor_type

    def __init__(self, sensor_type: str, builtin: bool = False, parent=None):
        super().__init__(parent)
        self._type     = sensor_type
        self._builtin  = builtin
        self._selected    = False
        self._delete_mode = False
        self.setFixedHeight(32)
        self.setCursor(Qt.PointingHandCursor)
        self._build()
        self._apply_style(False)

    def _build(self):
        h = QHBoxLayout(self)
        h.setContentsMargins(10, 0, 4, 0)
        h.setSpacing(6)

        self._dot = QFrame()
        self._dot.setFixedSize(6, 6)
        self._dot.setStyleSheet(
            "background-color: transparent; border: none;"
        )
        h.addWidget(self._dot)

        self._lbl = QLabel(self._type)
        self._lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        h.addWidget(self._lbl)

        self._btn_trash = QPushButton()
        self._btn_trash.setFixedSize(22, 22)
        self._btn_trash.setIcon(QIcon(f"{ICON_DIR}/clear.png"))
        self._btn_trash.setIconSize(Layout.ICON_SIZE_SM)
        self._btn_trash.setVisible(False)
        self._btn_trash.clicked.connect(lambda: self.delete_clicked.emit(self._type))
        h.addWidget(self._btn_trash)

    def set_delete_mode(self, active: bool):
        self._delete_mode = active
        self._btn_trash.setVisible(active and not self._builtin)

    def set_selected(self, selected: bool):
        self._apply_style(selected)

    def _apply_style(self, selected: bool):
        self._selected = selected
        bg = Colors.ACCENT_DIM if selected else "transparent"
        self.setStyleSheet(f"""
            SensorTypeCell {{
                background-color: {bg};
                border-bottom: 1px solid {Colors.DIVIDER};
            }}
            SensorTypeCell:hover {{
                background-color: {Colors.BG_CARD_HOVER if not selected else Colors.ACCENT_DIM};
            }}
        """)
        self._lbl.setStyleSheet(
            f"color: {Colors.ACCENT if selected else Colors.TEXT_WHITE};"
            f" font-size: 11px; font-weight: {'600' if selected else 'normal'};"
            f" background: transparent;"
        )
        self._btn_trash.setStyleSheet(f"""
            QPushButton {{
                background: transparent; border: none; border-radius: 3px;
            }}
            QPushButton:hover {{ background-color: {Colors.STATUS_RED}33; }}  /* 20% opacity */
            QPushButton:pressed {{ background-color: {Colors.STATUS_RED}66; }}  /* 40% opacity */
        """)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(self._type)
        super().mousePressEvent(event)


# ── Sensor instance cell ──────────────────────────────────────────────────────

class SensorCell(QFrame):
    clicked        = pyqtSignal(str)
    delete_clicked = pyqtSignal(str)

    STATUS_COLORS = {
        "all_passed":   Colors.STATUS_GREEN,
        "some_failed":  Colors.STATUS_YELLOW,
        "all_failed":   Colors.STATUS_RED,
        "never_tested": Colors.STATUS_BLUE,
    }

    def __init__(self, sensor_data: dict, parent=None):
        super().__init__(parent)
        self._data        = sensor_data
        self._selected    = False
        self._delete_mode = False
        self.setFixedHeight(Layout.SENSOR_CELL_HEIGHT)
        self.setCursor(Qt.PointingHandCursor)
        self._build()
        self._apply_style(False)

    def _build(self):
        h = QHBoxLayout(self)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(0)

        self._bar = QFrame()
        self._bar.setFixedWidth(3)
        h.addWidget(self._bar)

        self._lbl = QLabel(self._data.get("name", ""))
        self._lbl.setContentsMargins(12, 0, 8, 0)
        self._lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        h.addWidget(self._lbl)

        self._btn_trash = QPushButton()
        self._btn_trash.setFixedSize(28, 28)
        self._btn_trash.setIcon(QIcon(f"{ICON_DIR}/clear.png"))
        self._btn_trash.setIconSize(Layout.ICON_SIZE_SM)
        self._btn_trash.setVisible(False)
        self._btn_trash.clicked.connect(lambda: self.delete_clicked.emit(self._data["id"]))
        h.addWidget(self._btn_trash)
        h.setContentsMargins(0, 0, 4, 0)

        self._update_bar_color()

    def set_delete_mode(self, active: bool):
        self._delete_mode = active
        self._btn_trash.setVisible(active)

    def set_selected(self, selected: bool):
        self._apply_style(selected)

    def refresh(self, sensor_data: dict):
        self._data = sensor_data
        self._lbl.setText(sensor_data.get("name", ""))
        self._update_bar_color()

    def _status_key(self) -> str:
        tests = self._data.get("tests", [])
        if not tests:
            return "never_tested"
        statuses = [t.get("status", "Pending") for t in tests]
        if all(s == "Passed" for s in statuses):
            return "all_passed"
        if all(s == "Failed" for s in statuses):
            return "all_failed"
        if any(s == "Failed" for s in statuses):
            return "some_failed"
        return "never_tested"

    def _update_bar_color(self):
        color = self.STATUS_COLORS.get(self._status_key(), Colors.STATUS_BLUE)
        self._bar.setStyleSheet(f"background-color: {color}; border: none;")

    def _apply_style(self, selected: bool):
        self._selected = selected
        bg = Colors.BG_CARD_SEL if selected else Colors.BG_CARD
        self.setStyleSheet(f"""
            SensorCell {{
                background-color: {bg};
                border-bottom: 1px solid {Colors.DIVIDER};
            }}
            SensorCell:hover {{ background-color: {Colors.BG_CARD_HOVER}; }}
        """)
        self._lbl.setStyleSheet(
            f"color: {Colors.TEXT_WHITE if selected else Colors.TEXT_PRIMARY};"
            f" font-size: 13px; background-color: transparent;"
        )
        self._btn_trash.setStyleSheet(f"""
            QPushButton {{
                background: transparent; border: none; border-radius: 4px;
            }}
            QPushButton:hover {{ background-color: {Colors.STATUS_RED}33; }}  /* 20% opacity */
            QPushButton:pressed {{ background-color: {Colors.STATUS_RED}66; }}  /* 40% opacity */
        """)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(self._data["id"])
        super().mousePressEvent(event)


# ── ColSensors ────────────────────────────────────────────────────────────────

class ColSensors(QWidget):
    sensor_selected       = pyqtSignal(str)
    add_requested         = pyqtSignal()
    delete_requested      = pyqtSignal(str)
    add_type_requested    = pyqtSignal()
    edit_type_requested   = pyqtSignal(str)
    delete_type_requested = pyqtSignal(str)

    BUILTIN_TYPES: set = set()

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_1.ui", self)

        self._cells:            dict[str, SensorCell]     = {}
        self._type_cells:       dict[str, SensorTypeCell] = {}
        self._selected_id:      str | None = None
        self._selected_type:    str | None = None
        self._current_filter:   str | None = None
        self._all_sensors:      list[dict] = []
        self._delete_mode:      bool = False
        self._delete_type_mode: bool = False

        self._setup_heights()
        self._setup_styles()
        self._connect_signals()
        _install_drag_hint(self.wt_toolbar)

    # ── Public API ────────────────────────────────────────────────────────────

    def load_sensors(self, sensors: list[dict], types: list[str]):
        self._all_sensors = sensors
        self._rebuild_cells(sensors)
        try:
            import src.database.sensor_storage as db
            self.load_types(db.get_sensor_type_names())
        except Exception as exc:
            import traceback
            print(f"[ColSensors] load_sensors / load_types failed: {exc}\n{traceback.format_exc()}")

    def load_types(self, types: list[str]):
        layout = self.scroll_types_contents.layout()
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._type_cells.clear()

        for t in sorted(types):
            cell = SensorTypeCell(t, builtin=(t in self.BUILTIN_TYPES), parent=self)
            cell.set_delete_mode(self._delete_type_mode)
            cell.clicked.connect(self._on_type_clicked)
            cell.delete_clicked.connect(self.delete_type_requested)
            layout.addWidget(cell)
            self._type_cells[t] = cell

        layout.addStretch(1)

    def refresh_sensor(self, sensor_data: dict):
        cell = self._cells.get(sensor_data["id"])
        if cell:
            cell.refresh(sensor_data)

    def set_selected(self, sensor_id: str | None):
        if self._selected_id and self._selected_id in self._cells:
            self._cells[self._selected_id].set_selected(False)
        self._selected_id = sensor_id
        if sensor_id and sensor_id in self._cells:
            self._cells[sensor_id].set_selected(True)

    def remove_cell(self, sensor_id: str):
        cell = self._cells.pop(sensor_id, None)
        if cell:
            cell.deleteLater()
        if self._selected_id == sensor_id:
            self._selected_id = None

    # ── Private ───────────────────────────────────────────────────────────────

    def _open_add_type_dialog(self):
        from .dialog_add_sensor_type import AddSensorTypeDialog
        try:
            from src.tests import TESTS
            existing_tests = sorted(TESTS.keys())
        except Exception:
            existing_tests = []
        dlg = AddSensorTypeDialog(existing_tests=existing_tests, mode="add", parent=self)
        dlg.type_saved.connect(self._on_type_saved)
        dlg.exec_()

    def _open_edit_type_dialog(self):
        if not self._selected_type:
            return
        from .dialog_add_sensor_type import AddSensorTypeDialog
        try:
            from src.tests import TESTS
            existing_tests = sorted(TESTS.keys())
        except Exception:
            existing_tests = []
        try:
            import src.database.sensor_storage as db
            type_def = db.get_sensor_type(self._selected_type) or {}
            tests    = db.get_type_tests(self._selected_type)
        except Exception:
            type_def = {}
            tests    = []

        prefill = {
            "name":        type_def.get("sensor_type", self._selected_type),
            "description": type_def.get("description", ""),
            "params":      type_def.get("params", []),
            "detection":   type_def.get("detection", {}),
            "tests":       tests,
        }
        dlg = AddSensorTypeDialog(
            existing_tests = existing_tests,
            mode           = "edit",
            prefill        = prefill,
            parent         = self,
        )
        dlg.type_saved.connect(self._on_type_saved)
        dlg.exec_()

    def _on_type_saved(self, definition: dict):
        try:
            import src.database.sensor_storage as db
            self.load_types(db.get_sensor_type_names())
        except Exception as exc:
            import traceback
            print(f"[ColSensors] _on_type_saved error: {exc}\n{traceback.format_exc()}")

    def _on_type_clicked(self, sensor_type: str):
        if self._delete_type_mode:
            return
        if self._selected_type and self._selected_type in self._type_cells:
            self._type_cells[self._selected_type].set_selected(False)
        if self._selected_type == sensor_type:
            self._selected_type = None
            self._current_filter = None
            self.wt_filter_badge.setVisible(False)
            self.btn_edit_type.setEnabled(False)
            self._apply_search()
            return
        self._selected_type = sensor_type
        self._type_cells[sensor_type].set_selected(True)
        self.btn_edit_type.setEnabled(True)
        self._apply_filter(sensor_type)

    def _toggle_delete_mode(self, active: bool):
        self._delete_mode = active
        for cell in self._cells.values():
            cell.set_delete_mode(active)
        checked_style = f"""
            QPushButton {{
                background-color: {Colors.ACCENT_DIM};
                border: 1px solid {Colors.ACCENT};
                border-radius: 4px;
                padding: 4px;
            }}
            QPushButton:hover {{ background-color: {Colors.ACCENT_DIM}; }}
        """
        self.btn_delete.setStyleSheet(checked_style if active else Styles.BUTTON_ICON)

    def _toggle_delete_type_mode(self, active: bool):
        self._delete_type_mode = active
        for cell in self._type_cells.values():
            cell.set_delete_mode(active)
        checked_style = f"""
            QPushButton {{
                background-color: {Colors.ACCENT_DIM};
                border: 1px solid {Colors.ACCENT};
                border-radius: 4px;
                padding: 4px;
            }}
            QPushButton:hover {{ background-color: {Colors.ACCENT_DIM}; }}
        """
        self.btn_delete_type.setStyleSheet(checked_style if active else Styles.BUTTON_ICON)

    def _setup_heights(self):
        self.wt_types_toolbar.setFixedHeight(Layout.TOOLBAR_H)
        self.scroll_types.setFixedHeight(Layout.IMAGE_H - Layout.TOOLBAR_H)
        self.wt_toolbar.setFixedHeight(Layout.TOOLBAR_H)

    def _setup_styles(self):
        # Main container styles
        self.setStyleSheet(f"""
            QWidget {{ background-color: {Colors.BG_COLUMN}; }}
            QWidget#wt_types_toolbar {{
                background-color: {Colors.BG_TOOLBAR};
                border-bottom: 1px solid {Colors.BORDER};
            }}
            QWidget#wt_toolbar {{
                background-color: {Colors.BG_TOOLBAR};
                border-bottom: 1px solid {Colors.BORDER};
            }}
            QWidget#wt_filter_badge {{
                background-color: {Colors.BG_TOOLBAR};
                border-bottom: 1px solid {Colors.BORDER};
            }}
            QFrame#divider_line {{ background-color: {Colors.BORDER}; border: none; }}
            QScrollArea {{ border: none; background-color: {Colors.BG_CARD}; }}
            QWidget#scroll_sensors_contents {{ background-color: {Colors.BG_CARD}; }}
            QWidget#scroll_types_contents {{ background-color: {Colors.BG_COLUMN}; }}
            {Styles.SCROLLBAR}
        """)

        # Input field styles
        for inp in [self.input_search, self.input_types_search]:
            inp.setStyleSheet(f"""
                QLineEdit {{
                    background-color: {Colors.BG_INPUT};
                    border: 1px solid {Colors.BORDER};
                    border-radius: 4px;
                    color: {Colors.TEXT_PRIMARY};
                    padding: 2px 6px;
                    font-size: 11px;
                }}
                QLineEdit:focus {{ border-color: {Colors.ACCENT}; }}
            """)

        # Label styles
        self.lbl_filter_active.setStyleSheet(
            f"background-color: transparent; color: {Colors.TEXT_SECONDARY}; font-size: 11px;"
        )
        self.lbl_filter_value.setStyleSheet(
            f"background-color: transparent; color: {Colors.ACCENT}; font-size: 11px; font-weight: bold;"
        )

        # Button styles - all toolbar buttons get consistent sizing
        button_configs = [
            (self.btn_filter, Icons.FILTER()),
            (self.btn_add, Icons.ADD()),
            (self.btn_delete, Icons.CLEAR()),
            (self.btn_clear_filter, Icons.CLOSE()),
            (self.btn_add_type, Icons.ADD()),
            (self.btn_edit_type, Icons.EDIT()),
            (self.btn_delete_type, Icons.CLEAR()),
        ]
        
        for btn, icon in button_configs:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_SM)
            btn.setFixedSize(28, 28)  # Consistent size for all toolbar buttons
            btn.setStyleSheet(Styles.BUTTON_ICON)
        
        # Remove menu indicator from filter button (no small black arrow)
        self.btn_filter.setMenu(None)
        self.btn_filter.setStyleSheet(Styles.BUTTON_ICON)

        # Placeholder texts
        self.input_search.setPlaceholderText("Search sensors...")
        self.input_types_search.setPlaceholderText("Search types...")

    def _connect_signals(self):
        self.input_search.textChanged.connect(self._apply_search)
        self.input_types_search.textChanged.connect(self._apply_type_search)
        self.btn_clear_filter.clicked.connect(self._remove_filter)
        self.btn_add.clicked.connect(self.add_requested)
        self.btn_delete.toggled.connect(self._toggle_delete_mode)
        self.btn_add_type.clicked.connect(self._open_add_type_dialog)
        self.btn_edit_type.clicked.connect(self._open_edit_type_dialog)
        self.btn_delete_type.toggled.connect(self._toggle_delete_type_mode)
        self.delete_type_requested.connect(self._on_delete_type)

    def _on_delete_type(self, sensor_type: str):
        try:
            import src.database.sensor_storage as db
            
            sensors = db.get_sensors_by_type(sensor_type)
            if sensors:
                from PyQt5.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton
                
                dialog = QDialog(self)
                dialog.setWindowTitle("Cannot Delete Type")
                dialog.setFixedSize(360, 150)
                dialog.setStyleSheet(f"""
                    QDialog {{
                        background-color: {LC.BG_PANEL};
                    }}
                """)
                
                layout = QVBoxLayout(dialog)
                
                text = QLabel(f"Cannot delete type '{sensor_type}' because it is used by {len(sensors)} sensor(s).\n\nPlease delete or reassign these sensors first.")
                text.setWordWrap(True)
                text.setStyleSheet(f"color: {LC.TEXT}; font-size: 12px; background-color: transparent;")
                layout.addWidget(text)
                
                button_layout = QHBoxLayout()
                button_layout.addStretch()
                
                ok_button = QPushButton("OK")
                ok_button.setFixedSize(80, 28)
                ok_button.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {LC.ACCENT};
                        border: none;
                        border-radius: 3px;
                        color: white;
                        font-size: 12px;
                    }}
                    QPushButton:hover {{
                        background-color: {LC.ACCENT_HVR};
                    }}
                """)
                ok_button.clicked.connect(dialog.accept)
                
                button_layout.addWidget(ok_button)
                layout.addLayout(button_layout)
                
                dialog.exec_()
                return
            
            db.delete_sensor_type(sensor_type)
            db_types = db.get_sensor_type_names()
            self.load_types(db_types)
            if self._selected_type == sensor_type:
                self._selected_type = None
                
        except Exception as exc:
            import traceback
            print(f"[ColSensors] _on_delete_type error: {exc}\n{traceback.format_exc()}")

    def _rebuild_cells(self, sensors: list[dict]):
        layout = self.scroll_sensors_contents.layout()
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._cells.clear()
        for s in sensors:
            cell = self._make_cell(s)
            layout.addWidget(cell)
        layout.addStretch(1)

    def _make_cell(self, sensor_data: dict) -> SensorCell:
        cell = SensorCell(sensor_data, parent=self)
        cell.set_delete_mode(self._delete_mode)
        cell.clicked.connect(self._on_cell_clicked)
        cell.delete_clicked.connect(self.delete_requested)
        self._cells[sensor_data["id"]] = cell
        return cell

    def _on_cell_clicked(self, sensor_id: str):
        if self._delete_mode:
            return
        self.set_selected(sensor_id)
        self.sensor_selected.emit(sensor_id)

    def _apply_filter(self, filter_type: str):
        self._current_filter = filter_type
        self.lbl_filter_value.setText(filter_type)
        self.wt_filter_badge.setVisible(True)
        self._apply_search()

    def _remove_filter(self):
        self._current_filter = None
        self.wt_filter_badge.setVisible(False)
        if self._selected_type and self._selected_type in self._type_cells:
            self._type_cells[self._selected_type].set_selected(False)
        self._selected_type = None
        self.btn_edit_type.setEnabled(False)
        self._apply_search()

    def _apply_search(self):
        text = self.input_search.text().lower()
        for sid, cell in self._cells.items():
            data = cell._data
            name_match = text in data.get("name", "").lower()
            type_match = (
                self._current_filter is None
                or data.get("type", "").lower() == self._current_filter.lower()
            )
            cell.setVisible(name_match and type_match)

    def _apply_type_search(self):
        text = self.input_types_search.text().lower()
        for sensor_type, cell in self._type_cells.items():
            cell.setVisible(text in sensor_type.lower())