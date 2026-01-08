from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
import os

class SensorCard(QFrame):
    clicked = pyqtSignal(dict)
    
    def __init__(self, sensor_data, parent=None):
        super().__init__(parent)
        self.sensor_data = sensor_data
        
        uic.loadUi('./qt/sensor_card.ui', self)
        
        self.setCursor(Qt.PointingHandCursor)
        self.setup_card()
    
    def setup_card(self):
        self.name_label.setText(self.sensor_data['name'])
        
        if self.sensor_data.get('image_path') and os.path.exists(self.sensor_data['image_path']):
            pixmap = QPixmap(self.sensor_data['image_path'])
        
            self.image_label.setPixmap(pixmap)
    
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(self.sensor_data) #should emit id
    
    def enterEvent(self, event):
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(20)
        shadow.setColor(QColor(0, 0, 0, 60))
        shadow.setOffset(0, 4)
        self.setGraphicsEffect(shadow)
    
    def leaveEvent(self, event):
        self.setGraphicsEffect(None)