# sensor_page.py - Updated
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
import json
from datetime import datetime
import os
from sensor_card import SensorCard
from sensor_details import SensorDetails

class SensorPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi('./qt/sensor_page.ui', self)
        
        self.sensors_data = self.get_data('_mock.json')
        self.filtered_data = self.sensors_data.copy()

        self.columns = 4
        self.selected_sensor = None
        
        self.details_widget = SensorDetails()
        self.verticalLayout_2.addWidget(self.details_widget)
        self.details_widget.hide()
        
        self.setup_styles()
        self.connect_actions()
        
        self.update_cards_grid()
        
    def connect_actions(self):
        self.search_input.textChanged.connect(self.filter_sensors)
        self.sort_combo.currentTextChanged.connect(self.sort_sensors)

    def get_data(self, path):
        try:
            if not os.path.exists(path):
                return self.create_sample_data()
            with open(path, 'r') as f:
                data = json.load(f)
                sensors = data.get('sensors', [])
                return sensors
        except:
           pass
    
    def setup_styles(self):
        self.scroll_area.setStyleSheet("""
        QScrollArea {
                border: none;
                background-color: transparent;
            }
            QScrollBar:vertical {
                background: transparent;
                width: 8px;  /* Smaller width */
                margin: 0px;
                padding: 0px;
            }
            QScrollBar::handle:vertical {
                background-color: #c0c0c0;
                border-radius: 4px;
                min-height: 20px;
                margin: 0px;
            }
            QScrollBar::handle:vertical:hover {
                background-color: #a0a0a0;
            }
            QScrollBar::add-line:vertical, 
            QScrollBar::sub-line:vertical {
                background: transparent;
                border: none;
                height: 0px;
            }
            QScrollBar::add-page:vertical, 
            QScrollBar::sub-page:vertical {
                background: transparent;
            }
        """)

        self.search_input.setStyleSheet("""
            QLineEdit {
                padding: 8px 15px;
                border: 1px solid #ddd;
                border-radius: 8px;
                font-size: 14px;
                background-color: white;
            }
            QLineEdit:focus {
                border: 2px solid #2196F3;
            }
        """)
        self.sensor_types = ['camera', 'rfid', 'tactile', 'torque']
        self.sort_combo.addItems(self.sensor_types)
        self.sort_combo.setStyleSheet("""
            QComboBox {
                padding: 8px 15px;
                border: 1px solid #ddd;
                border-radius: 8px;
                font-size: 14px;
                background-color: white;
                min-width: 18px;
            }
        """)

        self.results_label.setStyleSheet("color: #666666; font-size: 12px; font-weight: bold;")
        self.no_results_label.setStyleSheet("font-size: 18px; color: #999; padding: 40px; text-align: center; font-weight: bold;")
        self.no_results_label.hide()

    
    def update_cards_grid(self):
        for i in reversed(range(self.cards_layout.count())):
            widget = self.cards_layout.itemAt(i).widget()
            if widget:
                widget.deleteLater()
        if not self.filtered_data:
            self.no_results_label.show()
            self.results_label.setText("0 sensors found")
            return
        self.no_results_label.hide()
        self.scroll_area.show()
        container_width = self.cards_container.width()
        card_width = 350
        spacing = 10
        self.columns = max(1, container_width // (card_width + spacing))
        for i, sensor_data in enumerate(self.filtered_data):
            row = i // self.columns
            col = i % self.columns
            card = SensorCard(sensor_data)
            card.clicked.connect(self.show_sensor_details)
            self.cards_layout.addWidget(card, row, col)
        count = len(self.filtered_data)
        total = len(self.sensors_data)
        self.results_label.setText(f"Showing {count} of {total} sensors")
    
    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_cards_grid()
    
    def filter_sensors(self, text):
        text = text.lower().strip()
        if not text:
            self.filtered_data = self.sensors_data.copy()
        else:
            self.filtered_data = [
                sensor for sensor in self.sensors_data
                if (text in sensor['name'].lower() or
                    text in sensor['type'].lower() or
                    text in sensor['location'].lower() or
                    (sensor.get('description') and text in sensor['description'].lower()))
            ]
        self.sort_sensors(self.sort_combo.currentText())
    
    def sort_sensors(self, sort_type):
        self.filtered_data.sort(key=lambda x: x.get('last_update', datetime.min), reverse=True)
        self.update_cards_grid()
    
    def show_sensor_details(self, sensor_data):
        self.details_widget.update_details(sensor_data)
        self.details_widget.show()
        self.selected_sensor = sensor_data
        self.update_cards_grid()
    
    def hide_details(self):
        self.details_widget.hide()
        self.selected_sensor = None
        self.update_cards_grid()