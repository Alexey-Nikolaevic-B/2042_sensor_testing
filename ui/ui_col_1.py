from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QFrame, QLabel, QSizePolicy, QMenu, QAction
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5 import uic

from .theme import Colors, Styles, Icons, Layout, QT_DIR


class SensorCell(QFrame):
    clicked = pyqtSignal(str)

    STATUS_COLORS = {
        "all_passed": Colors.STATUS_GREEN,
        "some_failed": Colors.STATUS_YELLOW,
        "all_failed": Colors.STATUS_RED,
        "never_tested": Colors.STATUS_BLUE,
    }

    def __init__(self, sensor_data: dict, parent=None):
        super().__init__(parent)
        self._data = sensor_data
        self._selected = False
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
            SensorCell:hover {{
                background-color: {Colors.BG_CARD_HOVER};
            }}
        """)
        self._lbl.setStyleSheet(
            f"color: {Colors.TEXT_WHITE if selected else Colors.TEXT_PRIMARY};"
            f" font-size: 13px; background-color: transparent;"
        )

    def set_selected(self, selected: bool):
        self._apply_style(selected)

    def refresh(self, sensor_data: dict):
        self._data = sensor_data
        self._lbl.setText(sensor_data.get("name", ""))
        self._update_bar_color()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(self._data["id"])
        super().mousePressEvent(event)



class ColSensors(QWidget):
    sensor_selected = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(f"{QT_DIR}/col_sensors.ui", self)

        self._cells: dict[str, SensorCell] = {}
        self._selected_id: str | None = None
        self._current_filter: str | None = None
        self._all_sensors: list[dict] = []

        self._setup_styles()
        self._setup_filter_menu()
        self._connect_signals()


    def load_sensors(self, sensors: list[dict], types: list[str]):
        self._all_sensors = sensors
        self._rebuild_filter_menu(types)
        self._rebuild_cells(sensors)

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


    def _setup_styles(self):
        self.setStyleSheet(f"""
            QWidget {{ background-color: {Colors.BG_COLUMN}; }}
            QWidget#wt_toolbar {{
                background-color: {Colors.BG_TOOLBAR};
                border-bottom: 1px solid {Colors.BORDER};
            }}
            QWidget#wt_filter_badge {{
                background-color: {Colors.BG_TOOLBAR};
                border-bottom: 1px solid {Colors.BORDER};
            }}
            QScrollArea {{ border: none; background-color: transparent; }}
            QWidget#scroll_sensors_contents {{ background-color: transparent; }}
            {Styles.SCROLLBAR}
        """)
        self.input_search.setStyleSheet(f"""
            QLineEdit {{
                background-color: {Colors.BG_INPUT};
                border: 1px solid {Colors.BORDER};
                border-radius: 4px;
                color: {Colors.TEXT_PRIMARY};
                padding: 3px 8px;
                font-size: 12px;
            }}
            QLineEdit:focus {{ border-color: {Colors.ACCENT}; }}
        """)
        self.lbl_filter_active.setStyleSheet(
            f"color: {Colors.TEXT_SECONDARY}; font-size: 11px;"
        )
        self.lbl_filter_value.setStyleSheet(
            f"color: {Colors.ACCENT}; font-size: 11px; font-weight: bold;"
        )
        for btn, icon in [
            (self.btn_filter,       Icons.FILTER()),
            (self.btn_add,          Icons.ADD()),
            (self.btn_clear_filter, Icons.CLOSE_BLK()),
        ]:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(Styles.BUTTON_ICON)

    def _setup_filter_menu(self):
        self._filter_menu = QMenu(self)
        self._filter_menu.setStyleSheet(f"""
            QMenu {{
                background-color: {Colors.BG_CARD};
                border: 1px solid {Colors.BORDER_LIGHT};
                color: {Colors.TEXT_PRIMARY};
            }}
            QMenu::item:selected {{ background-color: {Colors.BG_CARD_HOVER}; }}
        """)
        self.btn_filter.setMenu(self._filter_menu)
        self.btn_filter.setStyleSheet(
            Styles.BUTTON_ICON + "QPushButton::menu-indicator { image: none; }"
        )

    def _rebuild_filter_menu(self, types: list[str]):
        self._filter_menu.clear()
        for t in types:
            action = QAction(t, self)
            action.triggered.connect(lambda checked, _t=t: self._apply_filter(_t))
            self._filter_menu.addAction(action)

    def _connect_signals(self):
        self.input_search.textChanged.connect(self._apply_search)
        self.btn_clear_filter.clicked.connect(self._remove_filter)


    def _rebuild_cells(self, sensors: list[dict]):
        layout = self.scroll_sensors_contents.layout()
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._cells.clear()

        for s in sensors:
            cell = SensorCell(s, parent=self)
            cell.clicked.connect(self._on_cell_clicked)
            layout.addWidget(cell)
            self._cells[s["id"]] = cell

        layout.addStretch(1)

    def _on_cell_clicked(self, sensor_id: str):
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
