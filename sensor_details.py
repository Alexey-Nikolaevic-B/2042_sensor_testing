# sensor_details.py
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic

class SensorDetails(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi('./qt/sensor_details.ui', self)

        self.connect_signals()
        self.styles()

    def connect_signals(self):
        self.btn_close_details.clicked.connect(self.close)

    def styles(self):
        pass

    def close(self):
        self.setVisible(False)
        
    def update_details(self, sensor_data):
        self.sensor_name.setText(sensor_data['name'])
        self.type_label.setText(sensor_data['type'])
        self.status_label.setText(sensor_data['status'])
        self.description_label.setText(sensor_data.get('description', ''))
        
        if sensor_data.get('image_path'):
            pixmap = QPixmap(sensor_data['image_path'])
            if not pixmap.isNull():
                scaled_pixmap = pixmap.scaled(
                    self.image_label.size(),
                    Qt.IgnoreAspectRatio,
                    Qt.SmoothTransformation
                )
                self.image_label.setPixmap(scaled_pixmap)
        else:
            self.image_label.clear()