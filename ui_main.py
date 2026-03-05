from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt
from PyQt5 import uic

from ui_sensor_page import SensorPage
from ui_sensor_test import TestingWindow
from theme import Colors, Icons, Layout, QT_DIR


class Main_UI(QMainWindow):

    def __init__(self):
        super().__init__()

        self._is_maximized = False

        self._init_ui()
        self._init_pages()
        self._setup_styles()
        self._connect_signals()

        self._open_sensor_page()

    def _init_ui(self):
        uic.loadUi(f"{QT_DIR}/untitled.ui", self)
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.wt_sideabar_max.setVisible(False)

    def _init_pages(self):
        self.sensor_page = SensorPage()
        self.test_page   = TestingWindow()

    def _setup_styles(self):
        self.wt_top.setStyleSheet(
            f"QWidget {{ background-color: {Colors.BG_WINDOW}; }}"
        )
        self.wt_sideabar_min.setStyleSheet(
            f"QWidget {{ background-color: {Colors.BG_SIDEBAR}; }}"
        )
        self.wt_sideabar_max.setStyleSheet(
            f"QWidget {{ background-color: {Colors.BG_SIDEBAR}; }}"
        )
        self.wt_main.setStyleSheet(
            f"QWidget {{ background-color: {Colors.BG_MAIN}; }}"
        )

        for btn, icon in zip(
            [self.btn_minimize,    self.btn_maximize,    self.btn_close],
            [Icons.MINIMIZE(),     Icons.MAXIMIZE(),     Icons.CLOSE()],
        ):
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet("""
                QPushButton { background-color: transparent; border: none; padding: 5px; }
                QPushButton:hover { background-color: rgba(255,255,255,0.1); border-radius: 4px; }
                QPushButton:pressed { background-color: rgba(255,255,255,0.2); }
            """)

        sidebar_buttons = [
            self.btn_sidebar_menu,   self.btn_sidebar_sensor,  self.btn_sidebar_test,
            self.btn_sidebar_menu_2, self.btn_sidebar_sensor_2, self.btn_sidebar_test_2,
        ]
        sidebar_icons = [
            Icons.MENU(), Icons.SENSOR(), Icons.TEST_MENU(),
            Icons.MENU(), Icons.SENSOR(), Icons.TEST_MENU(),
        ]
        for btn, icon in zip(sidebar_buttons, sidebar_icons):
            btn.setIcon(icon)
            btn.setIconSize(Layout.ICON_SIZE_MD)
            btn.setStyleSheet("""
                QPushButton { background-color: transparent; border: none; padding: 5px; }
                QPushButton:hover { background-color: rgba(255,255,255,0.1); border-radius: 4px; }
                QPushButton:pressed { background-color: rgba(255,255,255,0.2); }
            """)

        self.app_icon.setPixmap(Icons.FAIL().pixmap(Layout.ICON_SIZE_MD))

    def _connect_signals(self):
        self.btn_minimize.clicked.connect(self.showMinimized)
        self.btn_maximize.clicked.connect(self._toggle_maximize)
        self.btn_close.clicked.connect(self.close)

        self.btn_sidebar_menu.clicked.connect(lambda: self._toggle_sidebar(False))
        self.btn_sidebar_menu_2.clicked.connect(lambda: self._toggle_sidebar(True))

        self.btn_sidebar_sensor.clicked.connect(self._goto_sensor_page)
        self.btn_sidebar_sensor_2.clicked.connect(self._goto_sensor_page)
        self.btn_sidebar_test.clicked.connect(self._goto_test_page)
        self.btn_sidebar_test_2.clicked.connect(self._goto_test_page)

        self.sensor_page.details_widget.signal_test_window.connect(
            self._open_testing_window
        )

    def _toggle_maximize(self):
        if self._is_maximized:
            self.showNormal()
        else:
            self.showFullScreen()
        self._is_maximized = not self._is_maximized

    def _toggle_sidebar(self, expanded: bool):
        self.wt_sideabar_min.setVisible(not expanded)
        self.wt_sideabar_max.setVisible(expanded)

    def _goto_sensor_page(self):
        self.btn_sidebar_sensor.setChecked(True)
        self.btn_sidebar_sensor_2.setChecked(True)
        self.stackedWidget.setCurrentIndex(0)

    def _goto_test_page(self):
        self.btn_sidebar_test.setChecked(True)
        self.btn_sidebar_test_2.setChecked(True)
        self.stackedWidget.setCurrentIndex(1)

    def _open_sensor_page(self):
        self.stackedWidget.insertWidget(0, self.sensor_page)
        self.stackedWidget.setCurrentIndex(0)

    def _open_testing_window(self, sensor_data: dict):
        self.stackedWidget.insertWidget(1, self.test_page)
        self.stackedWidget.setCurrentIndex(1)
        self.test_page.load_sensor(sensor_data)