from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
from datetime import datetime

from ui_sensor_card import SensorCard
from ui_sensor_details import SensorDetails
from sensor_repository import SensorRepository

from PyQt5.QtCore import pyqtSignal

icon_path = "./icon"


class SensorPage(QWidget):
    signal_closed_details = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self.repo = SensorRepository.instance()

        self.details_widget = SensorDetails()

        self.filtered_data = []
        self.selected_sensor = None
        self.current_filter = None
        self.cards_dict = {}

        self.init_ui()
        self.setup_signals()
        self.create_all_cards()
        self.setup_styles()
        self.connect_actions()
        self.apply_filters()

    def init_ui(self):
        uic.loadUi('./qt/sensor_page.ui', self)

        filter_menu = QMenu()
        for filter_type in self.repo.get_types():
            filter_menu.addAction(filter_type)

        self.btn_filter.setMenu(filter_menu)
        filter_menu.triggered.connect(self.filter_sensors)

        self.verticalLayout_2.addWidget(self.details_widget)
        self.details_widget.hide()

        self.no_results_label.hide()
        self.frame.hide()

    def setup_signals(self):
        self.details_widget.signal_closed_details.connect(self.hide_details)

        self.repo.sensor_added.connect(self._on_sensor_added)
        self.repo.sensor_updated.connect(self._on_sensor_updated)
        self.repo.sensor_deleted.connect(self._on_sensor_deleted)
        self.repo.sensors_loaded.connect(self._on_sensors_loaded)
        self.repo.test_updated.connect(self._on_test_updated)

    def _on_sensors_loaded(self):
        for sensor_id in list(self.cards_dict.keys()):
            if self.repo.get_sensor(sensor_id) is None:
                card = self.cards_dict.pop(sensor_id)
                card.deleteLater()

        for sensor_data in self.repo.all_sensors():
            if sensor_data["id"] not in self.cards_dict:
                self._add_card(sensor_data)

        self.apply_filters()

    def _on_sensor_added(self, sensor_data: dict):
        self._add_card(sensor_data)
        self.apply_filters()

    def _on_sensor_updated(self, sensor_data: dict):
        sensor_id = sensor_data["id"]
        card = self.cards_dict.get(sensor_id)
        if card:
            card.sensor_data = sensor_data
            card.setup_card()

        if (
            self.details_widget.isVisible()
            and self.selected_sensor
            and self.selected_sensor.get("id") == sensor_id
        ):
            self.details_widget.update_details(sensor_data)
            self.selected_sensor = sensor_data

        self.apply_filters()

    def _on_sensor_deleted(self, sensor_id: str):
        card = self.cards_dict.pop(sensor_id, None)
        if card:
            card.hide()
            card.deleteLater()

        if self.selected_sensor and self.selected_sensor.get("id") == sensor_id:
            self.hide_details()

        self.apply_filters()

    def _on_test_updated(self, sensor_id: str, test: dict):
        if (
            self.details_widget.isVisible()
            and self.selected_sensor
            and self.selected_sensor.get("id") == sensor_id
        ):
            fresh = self.repo.get_sensor(sensor_id)
            if fresh:
                self.details_widget.update_details(fresh)
                self.selected_sensor = fresh

    def _add_card(self, sensor_data: dict):
        sensor_id = sensor_data["id"]
        if sensor_id in self.cards_dict:
            return
        card = SensorCard(sensor_data)
        card.clicked.connect(self.show_sensor_details)
        self.cards_dict[sensor_id] = card

    def create_all_cards(self):
        for sensor_data in self.repo.all_sensors():
            self._add_card(sensor_data)

    def filter_sensors(self, action):
        self.current_filter = action.text()
        self.frame.show()
        self.btn_filter.hide()
        self.lbl_selected_filter.setText(self.current_filter)
        self.apply_filters()

    def remove_filter(self):
        self.current_filter = None
        self.frame.hide()
        self.btn_filter.show()
        self.apply_filters()

    def search_sensors(self):
        self.apply_filters()

    def apply_filters(self):
        search_text = self.search_input.text().lower()
        filtered = self.repo.all_sensors()

        if self.current_filter:
            filtered = [
                s for s in filtered
                if s.get("type", "").lower() == self.current_filter.lower()
            ]

        if search_text:
            filtered = [
                s for s in filtered
                if search_text in s.get("name", "").lower()
            ]

        filtered.sort(
            key=lambda x: x.get("last_update", datetime.min), reverse=True
        )

        self.filtered_data = filtered
        self.update_cards_grid()

    def show_sensor_details(self, sensor_data):
        fresh = self.repo.get_sensor(sensor_data["id"]) or sensor_data
        self.details_widget.update_details(fresh)
        self.details_widget.show()
        self.selected_sensor = fresh
        self.update_cards_grid()

    def close_details(self):
        self.details_widget.hide()
        self.update_cards_grid()

    def hide_details(self):
        self.details_widget.hide()
        self.selected_sensor = None
        self.update_cards_grid()

    def update_cards_grid(self):
        for card in self.cards_dict.values():
            card.hide()

        for i in reversed(range(self.cards_layout.count())):
            item = self.cards_layout.itemAt(i)
            if item.widget():
                self.cards_layout.removeWidget(item.widget())
            else:
                self.cards_layout.removeItem(item)

        for i in range(self.cards_layout.rowCount()):
            self.cards_layout.setRowStretch(i, 0)
        for i in range(self.cards_layout.columnCount()):
            self.cards_layout.setColumnStretch(i, 0)

        self.cards_layout.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        if not self.filtered_data:
            self.no_results_label.show()
            self.results_label.setText("0 sensors found")
            self.cards_container.setFixedSize(0, 0)
            return

        self.no_results_label.hide()
        self.scroll_area.show()

        if self.details_widget.isVisible():
            available_width = self.width() - self.details_widget.width()
        else:
            available_width = self.width() - 40

        card_width = 350
        horizontal_spacing = 10
        vertical_spacing = 30
        self.columns = max(1, available_width // (card_width + horizontal_spacing))

        rows = (len(self.filtered_data) + self.columns - 1) // self.columns

        container_width = self.columns * (card_width + horizontal_spacing) + 10
        card_height = 300
        container_height = rows * (card_height + vertical_spacing) + 20
        self.cards_container.setFixedSize(container_width, container_height)

        self.cards_layout.setHorizontalSpacing(horizontal_spacing)
        self.cards_layout.setVerticalSpacing(vertical_spacing)
        self.cards_layout.setContentsMargins(0, 0, 0, 0)

        for i, sensor_data in enumerate(self.filtered_data):
            row = i // self.columns
            col = i % self.columns
            card = self.cards_dict.get(sensor_data["id"])
            if card:
                card.show()
                card.setFixedSize(card_width, card_height)
                self.cards_layout.addWidget(card, row, col, Qt.AlignTop | Qt.AlignLeft)

        if self.columns > 0:
            self.cards_layout.setColumnStretch(self.columns, 1)
        if rows > 0:
            self.cards_layout.setRowStretch(rows, 1)

        count = len(self.filtered_data)
        total = self.repo.count()
        self.results_label.setText(f"Showing {count} of {total} sensors")

        self.cards_container.updateGeometry()
        self.scroll_area.viewport().update()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_cards_grid()

    def setup_styles(self):
        self.scroll_area.setStyleSheet("""
            QScrollArea { border: none; background-color: transparent; }
            QScrollBar:vertical { background: transparent; width: 8px; margin: 0px; }
            QScrollBar::handle:vertical { background-color: #c0c0c0; border-radius: 4px; min-height: 20px; }
            QScrollBar::handle:vertical:hover { background-color: #a0a0a0; }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; border: none; background: transparent; }
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }
        """)
        self.search_input.setStyleSheet("""
            QLineEdit { padding: 8px 15px; border: 1px solid #ddd; border-radius: 8px; font-size: 14px; background-color: white; }
            QLineEdit:focus { border: 2px solid #2196F3; }
        """)
        self.frame.setStyleSheet("""
            QWidget#frame {background-color: rgb(230, 230, 230); border-radius: 15px; border: 2px solid rgb(100, 100, 100); }
            QLabel { background-color: transparent; font-weight: bold; }
        """)
        button_style = """
            QPushButton { background-color: transparent; border: none; padding: 5px; }
            QPushButton:hover { background-color: rgba(100, 100, 100, 0.1); border-radius: 4px; }
            QPushButton:pressed { background-color: rgba(255, 255, 255, 0.2); }
        """
        self.btn_filter.setIcon(QIcon(f"{icon_path}/filter.png"))
        self.btn_filter.setStyleSheet(
            button_style + """
            QPushButton { padding: 8px 15px; border: 1px solid #ddd; border-radius: 8px; background-color: white; }
            QPushButton::menu-indicator { image: none; }
        """
        )
        self.btn_disable_filter.setIcon(QIcon(f"{icon_path}/close_black.png"))
        self.btn_disable_filter.setStyleSheet(button_style)
        self.results_label.setStyleSheet(
            "color: #666666; font-size: 12px; font-weight: bold;"
        )
        self.no_results_label.setStyleSheet(
            "font-size: 18px; color: #999; padding: 40px; text-align: center; font-weight: bold;"
        )

    def connect_actions(self):
        self.search_input.textChanged.connect(self.search_sensors)
        self.btn_disable_filter.clicked.connect(self.remove_filter)