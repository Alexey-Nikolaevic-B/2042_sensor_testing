from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic

icon_path = "./icon"

class SensorDetails(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
        self.setup_styles()

    def init_ui(self):
        uic.loadUi('./qt/sensor_details.ui', self)
        
    def setup_styles(self):
        self.setStyleSheet("""
            QWidget {
                background-color: #1e1e1e;
                border-left: 1px solid #333333;
            }
            
            QPushButton {
                border: 1px solid #444444;
                border-radius: 4px;
                padding: 6px 12px;
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
                padding: 10px 12px;
                border-bottom: 1px solid #333333;
                color: #cccccc;
                selection-background-color: transparent;
                selection-color: inherit;
            }
            
            QListWidget::item:selected {
                background-color: transparent;
                border-left: none;
                color: inherit;
            }
            
            QListWidget::item:hover {
                background-color: #2d2d2d;
            }
            
            QListWidget::item:focus {
                outline: none;
                border: none;
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
        """)



    def update_details(self, sensor_data):
        self.sensor_name.setText(sensor_data['name'])
        self.type_label.setText(sensor_data['type'])
        self.description_label.setText(sensor_data.get('description', ''))
        
        if sensor_data.get('image_path'):
            pixmap = QPixmap(sensor_data['image_path'])
            if not pixmap.isNull():
                scaled = pixmap.scaled(self.image_label.size(), Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
                self.image_label.setPixmap(scaled)
        else:
            self.image_label.clear()

        self.list_tests.clear()

        tests = [
            {"name": "Resolution Test", "date": "2024-01-15", "status": "Passed", "result": "3840x2160"},
            {"name": "Focus Calibration", "date": "2024-01-10", "status": "Passed", "result": "Optimal"},
            {"name": "Night Vision Test", "date": "2024-01-05", "status": "Failed", "result": "IR issue"},
            {"name": "Color Accuracy", "date": "2023-12-20", "status": "Passed", "result": "98%"}
        ]
        
        for test in tests:
            item_text = f"  {test['name']} - {test['date']}"
            item = QListWidgetItem(item_text)
            
            # Use different variable name to avoid conflict
            icon_file = "success.png" if test['status'] == "Passed" else "fail.png"
            icon = QIcon(f"{icon_path}/{icon_file}")
            item.setIcon(icon)
            
            if test['status'] == "Passed":
                item.setForeground(QColor("#27ae60"))
            elif test['status'] == "Failed":
                item.setForeground(QColor("#e74c3c"))
                
            item.setData(Qt.UserRole, test)
            self.list_tests.addItem(item)