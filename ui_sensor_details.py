from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic

from PyQt5.QtCore import pyqtSignal

icon_path = "./icon"
from ui_sensor_test import TestingWindow

class SensorDetails(QWidget):
    signal_test_window = pyqtSignal(dict)
    signal_closed_details = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self.sensor_data = None

        self.init_ui()
        self.setup_styles()
        self.setup_connections()

    def setup_connections(self):
        self.btn_test_sensor.clicked.connect(self.open_testing_window)
        self.btn_close_details.clicked.connect(self.close)

    def init_ui(self):
        uic.loadUi('./qt/sensor_details.ui', self)
        
    def update_details(self, sensor_data):
        self.sensor_data = sensor_data
        self.sensor_name.setText(sensor_data['name'])
        self.type_label.setText(sensor_data['type'])
        self.description_label.setText(sensor_data.get('description', ''))
        
        if sensor_data.get('image_path'):
            pixmap = QPixmap(sensor_data['image_path'])
            if not pixmap.isNull():
                scaled = pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.image_label.setPixmap(scaled)
        else:
            self.image_label.clear()

        self.list_tests.clear()

        tests = self.sensor_data['tests']
        
        for test in tests:
            item_text = f"  {test['name']} - {test['date']}"
            item = QListWidgetItem(item_text)
            
            icon_file = "success.png" if test['status'] == "Passed" else "fail.png"
            icon = QIcon(f"{icon_path}/{icon_file}")
            item.setIcon(icon)
                
            item.setData(Qt.UserRole, test)
            self.list_tests.addItem(item)     
    
    def open_testing_window(self):
        self.signal_test_window.emit(self.sensor_data)

    def close(self):
        self.signal_closed_details.emit()
        self.hide()
        
    def setup_styles(self):
        self.setStyleSheet("""
        QWidget {
            background-color: #1e1e1e;
        }
        
        QPushButton {
            border: 1px solid #444444;
            border-radius: 4px;
            background-color: #2d2d2d;
            color: #cccccc;
            outline: none;
        }
        
        QPushButton:hover {
            background-color: #3a3a3a;
            border-color: #555555;
        }
        
        QPushButton:pressed {
            background-color: #252525;
        }
        
        QPushButton#btn_export {
            background-color: #25394d;
            border-color: #3498db;
            color: #5dade2;
        }
        
        QPushButton#btn_export:hover {
            background-color: #2c3e50;
        }
        
        QPushButton#btn_new_test {
            background-color: #1e3a2a;
            border-color: #2ecc71;
            color: #27ae60;
        }
        
        QPushButton#btn_new_test:hover {
            background-color: #225633;
        }
        
        QPushButton#btn_close_details {
            background-color: #2d2d2d;
            border: 1px solid #444444;
            color: #aaaaaa;
            font-weight: bold;
        }
        
        QPushButton#btn_close_details:hover {
            background-color: #3a3a3a;
            color: #cccccc;
        }
        
        QListWidget {
            border: 1px solid #333333;
            background-color: #252525;
            alternate-background-color: #2a2a2a;
            outline: none;
        }
        
        QListWidget::item {
            border-bottom: 1px solid #333333;
            color: #cccccc;
        }
        
        QListWidget::item:hover {
            background-color: #2d2d2d;
        }
        
        QListWidget::item:selected:hover {
            background-color: #454545;
        }
        
        /* Fix for the white selection text */
        QListWidget::item:selected {
            color: #ffffff;
        }
        
        QScrollBar:vertical {
            background: transparent;
            width: 0px;
        }
        
        QScrollBar::handle:vertical {
            background: transparent;
        }
        
        QScrollBar::add-line:vertical, 
        QScrollBar::sub-line:vertical {
            height: 0px;
            background: transparent;
        }
        
        QScrollBar::add-page:vertical, 
        QScrollBar::sub-page:vertical {
            background: transparent;
        }
        
        QLabel#sensor_name {
            font-size: 20px;
            font-weight: bold;
            color: white;
        }
        
        QLabel#type_label, QLabel#id_label, QLabel#status_label {
            font-weight: bold;
            color: #dddddd;
        }
        
        QLabel#status_label {
            color: #27ae60;
        }
        
        QLabel#description_label {
            color: #aaaaaa;
            padding: 5px 0px;
        }
        
        QLabel[style*="color: #666666"] {
            color: #999999;
            background-color: transparent;
        }
        
        QLabel#image_label {
            background-color: #252525;
            border: 1px solid #333333;
        }
        QLabel#horizontalLayout {
            background-color: #252525;
            border: 1px solid #333333;
        }
    """)
        
        button_style = """
            QPushButton { background-color: transparent; border: none; padding: 5px; }
            QPushButton:hover { background-color: rgba(255, 255, 255, 0.1); border-radius: 4px; }
            QPushButton:pressed { background-color: rgba(255, 255, 255, 0.2); }
        """

        self.btn_test_sensor.setStyleSheet(button_style)
        self.btn_export.setStyleSheet(button_style)
        
        self.btn_test_sensor.setIcon(QIcon("icon/test_menu.png"))
        self.btn_export.setIcon(QIcon("icon/export.png"))