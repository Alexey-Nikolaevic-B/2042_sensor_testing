from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtGui import QIcon
from PyQt5 import uic
from ui_sensor_page import SensorPage

icon_path = "./icon"

class Main_UI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.is_maximized = False
        self.init_ui()
        self.setup_styles()
        self.connect_actions()
        self.setup_sensor_page()

    def init_ui(self):
        uic.loadUi('./qt/untitled.ui', self)
        self.setWindowFlag(Qt.FramelessWindowHint)
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.wt_sideabar_max.setVisible(False)

    def setup_sensor_page(self):
        self.sensor_page = SensorPage(parent=self)
        self.stackedWidget.insertWidget(0, self.sensor_page)

    def setup_styles(self):
        button_style = """
            QPushButton { background-color: transparent; border: none; padding: 5px; }
            QPushButton:hover { background-color: rgba(255, 255, 255, 0.1); border-radius: 4px; }
            QPushButton:pressed { background-color: rgba(255, 255, 255, 0.2); }
        """
        
        self.wt_top.setStyleSheet("QWidget {background-color : rgb(60, 60, 60)}")
        self.wt_sideabar_min.setStyleSheet("QWidget {background-color : rgb(177, 177, 177)}")
        self.wt_sideabar_max.setStyleSheet("QWidget {background-color : rgb(177, 177, 177)}")
        self.wt_main.setStyleSheet("QWidget {background-color : rgb(230, 230, 230)}")
        
        top_buttons = [self.btn_minimize, self.btn_maximize, self.btn_close]
        top_icons = ["minimize.png", "maximize.png", "close.png"]
        
        for button, icon in zip(top_buttons, top_icons):
            button.setIcon(QIcon(f"{icon_path}/{icon}"))
            button.setStyleSheet(button_style)
        
        sidebar_buttons = [
            self.btn_sidebar_menu, self.btn_sidebar_sensor, self.btn_sidebar_test,
            self.btn_sidebar_menu_2, self.btn_sidebar_sensor_2, self.btn_sidebar_test_2
        ]
        sidebar_icons = ["menu.png", "sensor.png", "minimize.png"] * 2
        
        for button, icon in zip(sidebar_buttons, sidebar_icons):
            button.setIcon(QIcon(f"{icon_path}/{icon}"))
            button.setStyleSheet(button_style)

    def connect_actions(self):
        self.btn_minimize.clicked.connect(self.showMinimized)
        self.btn_maximize.clicked.connect(self.toggle_maximize)
        self.btn_close.clicked.connect(self.close)
        
        self.btn_sidebar_menu.clicked.connect(lambda: self.toggle_sidebar(False))
        self.btn_sidebar_menu_2.clicked.connect(lambda: self.toggle_sidebar(True))
        
        self.btn_sidebar_sensor.clicked.connect(self.goto_main_sensor)
        self.btn_sidebar_sensor_2.clicked.connect(self.goto_main_sensor)
        
        self.btn_sidebar_test.clicked.connect(self.goto_main_test)
        self.btn_sidebar_test_2.clicked.connect(self.goto_main_test)

    def toggle_maximize(self):
        if not self.is_maximized:
            self.showFullScreen()
            self.is_maximized = True
        else:
            self.showNormal()
            self.is_maximized = False

    def goto_main_sensor(self):
        self.btn_sidebar_sensor.setChecked(True)
        self.btn_sidebar_sensor_2.setChecked(True)
        self.stackedWidget.setCurrentIndex(0)

    def goto_main_test(self):
        self.btn_sidebar_test.setChecked(True)
        self.btn_sidebar_test_2.setChecked(True)
        self.stackedWidget.setCurrentIndex(1)

    def toggle_sidebar(self, toggle):
        self.wt_sideabar_min.setVisible(toggle)
        self.wt_sideabar_max.setVisible(not toggle)