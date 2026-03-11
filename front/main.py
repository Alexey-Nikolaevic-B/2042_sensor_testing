from PyQt5.QtWidgets import QMainWindow, QSizePolicy
from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

from ._theme import Colors, Styles, Icons, Layout, QT_DIR
from .widget_col_1 import ColSensors
from .widget_col_2 import ColDetails
from .widget_col_3 import ColTests
from .widget_col_4 import ColCapture
from .logic_sensor_repository import SensorRepository
from .dialog_add_sensor import AddSensorDialog


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
        self.col_1 = ColSensors()
        self.col_2 = ColDetails()
        self.col_3   = ColTests()
        self.col_4 = ColCapture()

        self.test_page = self.col_3

        for col in (self.col_1, self.col_2,
                    self.col_3, self.col_4):
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

        self.col_1.sensor_selected.connect(self._on_sensor_selected)
        self.col_1.add_requested.connect(self._on_add_sensor)
        self.col_1.delete_requested.connect(self._on_delete_sensor)


        self.col_3._runner_log_forward = self.col_4.append_log

    def _load_all_sensors(self):
        repo = SensorRepository.instance()
        sensors = repo.all_sensors()
        types   = repo.get_types()
        self.col_1.load_sensors(sensors, types)


        repo.sensor_added.connect(lambda _: self._reload_sensors())
        repo.sensor_updated.connect(lambda _: self._reload_sensors())
        repo.test_updated.connect(self._on_test_updated)


    def _on_sensor_selected(self, sensor_id: str):
        repo = SensorRepository.instance()
        sensor = repo.get_sensor(sensor_id)
        if not sensor:
            return
        data = sensor
        self.col_2.load_sensor(data)
        self.col_3.load_sensor(data)

    def _reload_sensors(self):
        repo = SensorRepository.instance()
        prev_id = self.col_1._selected_id
        self.col_1.load_sensors(repo.all_sensors(), repo.get_types())
        if prev_id:
            self.col_1.set_selected(prev_id)

            sensor = repo.get_sensor(prev_id)
            if sensor:
                self.col_2.load_sensor(sensor)
                self.col_3.load_sensor(sensor)

    def _on_test_updated(self, sensor_id: str):
        repo   = SensorRepository.instance()
        sensor = repo.get_sensor(sensor_id)
        if sensor:
            self.col_1.refresh_sensor(sensor)

    def set_core(self, core) -> None:
        self._core = core

    def _on_add_sensor(self):
        core = getattr(self, '_core', None)
        dlg = AddSensorDialog(parent=self, core=core)
        dlg.sensor_saved.connect(self._on_sensor_saved)
        dlg.exec_()

    def _on_sensor_edited(self, sensor_dict: dict):
        repo = SensorRepository.instance()
        sensor_id = sensor_dict.get('id')
        if not sensor_id:
            return
        try:
            updated = repo.update_sensor(sensor_id, sensor_dict)
            self.col_1.refresh_sensor(updated)
        except Exception as e:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.warning(self, 'Update failed', str(e))

    def _on_delete_sensor(self, sensor_id: str):
        repo = SensorRepository.instance()
        try:
            was_selected = (self.col_1._selected_id == sensor_id)
            repo.delete_sensor(sensor_id)
            self.col_1.remove_cell(sensor_id)
            if was_selected:
                self.col_2.clear()
                self.col_3.clear()
        except Exception as e:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.warning(self, 'Delete failed', str(e))

    def _on_sensor_saved(self, sensor_dict: dict):
        repo = SensorRepository.instance()
        try:
            repo.add_sensor(sensor_dict)
            # repo.sensor_added signal triggers _reload_sensors automatically
        except Exception as e:
            from PyQt5.QtWidgets import QMessageBox
            QMessageBox.warning(self, "Save failed", str(e))

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