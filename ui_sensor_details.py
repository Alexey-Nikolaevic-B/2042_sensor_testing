from PyQt5.QtWidgets import QWidget, QListWidgetItem
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

from theme import Colors, Styles, Icons, Layout, QT_DIR


class SensorDetails(QWidget):

    signal_test_window    = pyqtSignal(dict)
    signal_closed_details = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        self._sensor_data = None

        self._init_ui()
        self._setup_styles()
        self._connect_signals()

    def _init_ui(self):
        uic.loadUi(f"{QT_DIR}/sensor_details.ui", self)

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QWidget {{
                background-color: {Colors.BG_PRIMARY};
            }}
            QPushButton {{
                border: 1px solid {Colors.BORDER_LIGHT};
                border-radius: 4px;
                background-color: {Colors.BG_ELEVATED};
                color: {Colors.TEXT_PRIMARY};
                outline: none;
            }}
            QPushButton:hover {{
                background-color: {Colors.BG_HOVER};
                border-color: {Colors.BORDER_FOCUS};
            }}
            QPushButton:pressed {{
                background-color: {Colors.BG_SECONDARY};
            }}
            QPushButton#btn_close_details {{
                background-color: {Colors.BG_ELEVATED};
                border: 1px solid {Colors.BORDER_LIGHT};
                color: {Colors.TEXT_SECONDARY};
                font-weight: bold;
            }}
            QPushButton#btn_close_details:hover {{
                background-color: {Colors.BG_HOVER};
                color: {Colors.TEXT_PRIMARY};
            }}
            QLabel#sensor_name {{
                font-size: 20px;
                font-weight: bold;
                color: {Colors.TEXT_WHITE};
            }}
            QLabel#type_label, QLabel#id_label, QLabel#status_label {{
                font-weight: bold;
                color: {Colors.TEXT_BRIGHT};
            }}
            QLabel#status_label {{
                color: {Colors.STATUS_PASS};
            }}
            QLabel#description_label {{
                color: {Colors.TEXT_SECONDARY};
                padding: 5px 0px;
            }}
            QLabel#image_label {{
                background-color: {Colors.BG_SECONDARY};
                border: 1px solid {Colors.BORDER};
            }}
            {Styles.LIST_WIDGET}
            {Styles.SCROLLBAR_HIDDEN}
        """)

        self.btn_test_sensor.setStyleSheet(Styles.BUTTON_ICON)
        self.btn_export.setStyleSheet(Styles.BUTTON_ICON)

        self.btn_test_sensor.setIcon(Icons.TEST_MENU())
        self.btn_test_sensor.setIconSize(Layout.ICON_SIZE_MD)
        self.btn_export.setIcon(Icons.EXPORT())
        self.btn_export.setIconSize(Layout.ICON_SIZE_MD)

    def _connect_signals(self):
        self.btn_test_sensor.clicked.connect(self._on_test_sensor_clicked)
        self.btn_close_details.clicked.connect(self._on_close_clicked)

    def load_sensor(self, sensor_data: dict):
        self._sensor_data = sensor_data

        self.sensor_name.setText(sensor_data["name"])
        self.type_label.setText(sensor_data["type"])
        self.description_label.setText(sensor_data.get("description", ""))

        self.image_label.clear()
        image_path = sensor_data.get("image_path", "")
        if image_path:
            pixmap = QPixmap(image_path)
            if not pixmap.isNull():
                scaled = pixmap.scaled(
                    self.image_label.size(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
                self.image_label.setPixmap(scaled)

        self.list_tests.clear()
        for test in sensor_data.get("tests", []):
            item = QListWidgetItem(f"  {test['name']} - {test['date']}")
            item.setIcon(Icons.for_status(test["status"]))
            item.setData(Qt.UserRole, test)
            self.list_tests.addItem(item)

    def _on_test_sensor_clicked(self):
        if self._sensor_data:
            self.signal_test_window.emit(self._sensor_data)

    def _on_close_clicked(self):
        self.signal_closed_details.emit()
        self.hide()