from PyQt5.QtWidgets import QMainWindow, QSizePolicy
from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

from .theme import Colors, Styles, Icons, Layout, QT_DIR
from .ui_col_1 import ColSensors
from .ui_col_2 import ColDetails
from .ui_col_3 import ColTests
from .ui_col_4 import ColCapture
from sensor_repository import SensorRepository


class Main_UI(QMainWindow):

    def __init__(self):
        super().__init__()
        self._is_maximized  = False
        self._drag_pos      = None

        self._init_ui()
        self._build_columns()
        self._setup_styles()
        self._connect_signals()
        self._load_all_sensors()

    def _init_ui(self):
        uic.loadUi(f"{QT_DIR}/main.ui", self)
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_TranslucentBackground)

    def _build_columns(self):
        self.col_sensors = ColSensors()
        self.col_details = ColDetails()
        self.col_tests   = ColTests()
        self.col_capture = ColCapture()

        self.test_page = self.col_tests

        for col in (self.col_sensors, self.col_details,
                    self.col_tests, self.col_capture):
            col.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.splitter_main.addWidget(col)

        self.splitter_main.setSizes([300, 300, 300, 300])

    def _setup_styles(self):
        self.setStyleSheet(f"""
            QMainWindow, QWidget#centralwidget {{
                background-color: {Colors.BG_APP};
                border-radius: 8px;
            }}
            QWidget#wt_titlebar {{
                background-color: {Colors.BG_TITLEBAR};
                border-bottom: 1px solid {Colors.BORDER};
            }}
            QLabel#lbl_app_title {{
                color: {Colors.TEXT_SECONDARY};
                font-size: 12px;
                font-weight: bold;
            }}
            QSplitter::handle {{
                background-color: {Colors.SPLITTER};
                width: 1px;
            }}
        """)
        self.app_icon.setPixmap(Icons.SENSOR().pixmap(Layout.ICON_SIZE_MD))
        for btn, icon in [
            (self.btn_minimize, Icons.MINIMIZE()),
            (self.btn_maximize, Icons.MAXIMIZE()),
            (self.btn_close,    Icons.CLOSE()),
        ]:
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet(Styles.BUTTON_ICON)

    def _connect_signals(self):
        self.btn_minimize.clicked.connect(self.showMinimized)
        self.btn_maximize.clicked.connect(self._toggle_maximize)
        self.btn_close.clicked.connect(self.close)

        self.col_sensors.sensor_selected.connect(self._on_sensor_selected)

        self.col_tests._runner_log_forward = self.col_capture.append_log

    def _load_all_sensors(self):
        repo = SensorRepository.instance()
        sensors = repo.all_sensors()
        types   = repo.get_types()
        self.col_sensors.load_sensors(sensors, types)

        repo.sensor_added.connect(lambda _: self._reload_sensors())
        repo.sensor_updated.connect(lambda _: self._reload_sensors())
        repo.test_updated.connect(self._on_test_updated)

    def _on_sensor_selected(self, sensor_id: str):
        repo = SensorRepository.instance()
        sensor = repo.get_sensor(sensor_id)
        if not sensor:
            return
        data = sensor
        self.col_details.load_sensor(data)
        self.col_tests.load_sensor(data)

    def _reload_sensors(self):
        repo = SensorRepository.instance()
        self.col_sensors.load_sensors(repo.all_sensors(), repo.get_types())

    def _on_test_updated(self, sensor_id: str):
        repo   = SensorRepository.instance()
        sensor = repo.get_sensor(sensor_id)
        if sensor:
            self.col_sensors.refresh_sensor(sensor)

    def _toggle_maximize(self):
        if self._is_maximized:
            self.showNormal()
        else:
            self.showMaximized()
        self._is_maximized = not self._is_maximized

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self.wt_titlebar.geometry().contains(event.pos()):
                self._drag_pos = event.globalPos() - self.frameGeometry().topLeft()

    def mouseMoveEvent(self, event):
        if self._drag_pos and event.buttons() == Qt.LeftButton:
            self.move(event.globalPos() - self._drag_pos)

    def mouseReleaseEvent(self, event):
        self._drag_pos = None