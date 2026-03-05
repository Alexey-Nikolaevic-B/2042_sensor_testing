import os

from PyQt5.QtWidgets import QFrame, QGraphicsDropShadowEffect
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap, QColor
from PyQt5 import uic

from theme import Colors, Styles, Layout, QT_DIR


class SensorCard(QFrame):

    clicked = pyqtSignal(str)

    def __init__(self, sensor_data: dict, parent=None):
        super().__init__(parent)

        self.sensor_data = sensor_data

        self._init_ui()
        self._init_state()
        self._setup_styles()

    def _init_ui(self):
        uic.loadUi(f"{QT_DIR}/sensor_card.ui", self)
        self.setCursor(Qt.PointingHandCursor)

    def _init_state(self):
        self.name_label.setText(self.sensor_data["name"])

        image_path = self.sensor_data.get("image_path", "")
        if image_path and os.path.exists(image_path):
            self.image_label.setPixmap(QPixmap(image_path))

    def _setup_styles(self):
        self.name_label.setStyleSheet(Styles.CARD_NAME_LABEL)
        self.image_label.setStyleSheet(Styles.CARD_IMAGE_LABEL)

    def refresh(self, sensor_data: dict):
        self.sensor_data = sensor_data
        self._init_state()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(self.sensor_data["id"])

    def enterEvent(self, event):
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(20)
        shadow.setColor(QColor(0, 0, 0, 60))
        shadow.setOffset(0, 4)
        self.setGraphicsEffect(shadow)

    def leaveEvent(self, event):
        self.setGraphicsEffect(None)