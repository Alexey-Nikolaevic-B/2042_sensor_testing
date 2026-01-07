from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import pyqtSignal
from PyQt5.uic import loadUi
from PyQt5 import QtCore

from PyQt5.QtCore import QTimer

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainScreen, self).__init__()
        self.init_ui()

        self.btn_sidebar_menu.clicked.connect(lambda: self.toggle_sidebar(False))
        self.btn_sidebar_menu_2.clicked.connect(lambda: self.toggle_sidebar(True))

        self.btn_sidebar_sensor.clicked.connect(self.goto_main_sensor)
        self.btn_sidebar_sensor_2.clicked.connect(self.goto_main_sensor)

        self.btn_sidebar_test.clicked.connect(self.goto_main_test)
        self.btn_sidebar_test_2.clicked.connect(self.goto_main_test)

    def goto_main_sensor(self):
        self.btn_sidebar_sensor.setChecked(True)
        self.btn_sidebar_sensor_2.setChecked(True)

    def goto_main_test(self):
        self.btn_sidebar_test.setChecked(True)
        self.btn_sidebar_test_2.setChecked(True)

    def toggle_sidebar(self, toggle):
        self.wt_sideabar_min.setVisible(toggle)
        self.wt_sideabar_max.setVisible(not toggle)

    def init_ui(self):
        loadUi(('./qt/main.ui'), self)
        
        # DEBUG: Check if widgets exist
        print(f"wt_sideabar_min exists: {hasattr(self, 'wt_sideabar_min')}")
        print(f"wt_sideabar_max exists: {hasattr(self, 'wt_sideabar_max')}")
        
        # DEBUG: Check widget types
        if hasattr(self, 'wt_sideabar_min'):
            print(f"wt_sideabar_min type: {type(self.wt_sideabar_min)}")
            print(f"wt_sideabar_min is visible: {self.wt_sideabar_min.isVisible()}")
        
        # ... rest of init_ui

        self.setWindowFlags(
            Qt.FramelessWindowHint | 
            Qt.Window | 
            Qt.WindowSystemMenuHint |
            Qt.WindowMinimizeButtonHint |
            Qt.WindowMaximizeButtonHint |
            Qt.WindowCloseButtonHint
        )


                
        self.setWindowFlag(Qt.FramelessWindowHint)

        style_btn = "QPushButton {color: rgb(0, 0, 0); background-color : rgb(200, 200, 200)} QPushButton::hover {background-color: rgb(255, 255, 255)}"

        style_1 = "QWidget {background-color : rgb(230, 230, 230)}"
        style_2 = "QWidget {background-color : rgb(177, 177, 177)}"
        style_3 = "QWidget {background-color : rgb(60, 60, 60)}"


        # Widgets
        self.wt_top.setStyleSheet(style_3)

        self.wt_sideabar_min.setStyleSheet(style_2)
        self.wt_sideabar_max.setStyleSheet(style_2)

        self.wt_main.setStyleSheet(style_1)


        # Sidebar
        self.btn_sidebar_menu.setStyleSheet(style_btn)
        self.btn_sidebar_menu_2.setStyleSheet(style_btn)

        self.btn_sidebar_sensor.setStyleSheet(style_btn)
        self.btn_sidebar_sensor_2.setStyleSheet(style_btn)

        self.btn_sidebar_test.setStyleSheet(style_btn)
        self.btn_sidebar_test_2.setStyleSheet(style_btn)