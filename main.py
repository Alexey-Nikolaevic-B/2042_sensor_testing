import sys
from PyQt5.QtWidgets import QApplication

from PyQt5.QtWidgets import QDialog
from PyQt5.QtCore import pyqtSignal
from PyQt5.uic import loadUi
from PyQt5 import QtCore

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from PyQt5.QtGui import QIcon
from PyQt5 import uic


from sensor_page import SensorPage
from test_page import TestPage

class Main_UI(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)

        self.init_ui()

        self.sensor_page = SensorPage(parent=self)
        self.test_page = TestPage(parent=self)
        
        self.stackedWidget.insertWidget(0, self.sensor_page)
        self.stackedWidget.insertWidget(1, self.test_page)

        self.is_maximized = False

        # Top
        self.btn_minimize.clicked.connect(self.minimize_window)
        self.btn_maximize.clicked.connect(self.maximize_window)
        self.btn_close.clicked.connect(self.close_window)

        # Sidebar
        self.btn_sidebar_menu.clicked.connect(lambda: self.toggle_sidebar(False))
        self.btn_sidebar_menu_2.clicked.connect(lambda: self.toggle_sidebar(True))

        self.btn_sidebar_sensor.clicked.connect(self.goto_main_sensor)
        self.btn_sidebar_sensor_2.clicked.connect(self.goto_main_sensor)

        self.btn_sidebar_test.clicked.connect(self.goto_main_test)
        self.btn_sidebar_test_2.clicked.connect(self.goto_main_test)


    def minimize_window(self):
        self.showMinimized()

    def maximize_window(self):
        if not self.is_maximized:            
            self.showFullScreen()
            self.is_maximized = True    
            
        else:
            self.showNormal()
            self.is_maximized = False

    def close_window(self):
        self.close()

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

    def init_ui(self):
        uic.loadUi('./qt/untitled.ui', self)
        self.setWindowFlag(Qt.FramelessWindowHint)

        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_TranslucentBackground)

        self.wt_sideabar_max.setVisible(False)

        style_btn = """
        QPushButton {
            background-color: transparent;
            border: none;
            padding: 5px;
        }
        QPushButton:hover {
            background-color: rgba(255, 255, 255, 0.1);
            border-radius: 4px;
        }
        QPushButton:pressed {
            background-color: rgba(255, 255, 255, 0.2);
        }
        """

        style_1 = "QWidget {background-color : rgb(230, 230, 230)}"
        style_2 = "QWidget {background-color : rgb(177, 177, 177)}"
        style_3 = "QWidget {background-color : rgb(60, 60, 60)}"


        # Widgets
        self.wt_top.setStyleSheet(style_3)

        self.wt_sideabar_min.setStyleSheet(style_2)
        self.wt_sideabar_max.setStyleSheet(style_2)

        self.wt_main.setStyleSheet(style_1)

        # Top
        self.btn_minimize.setIcon(QIcon("./img/minimize.png"))
        self.btn_maximize.setIcon(QIcon("./img/maximize.png"))
        self.btn_close.setIcon(QIcon("./img/close.png"))

        self.btn_minimize.setStyleSheet(style_btn)
        self.btn_maximize.setStyleSheet(style_btn)
        self.btn_close.setStyleSheet(style_btn)

        # Sidebar
        self.btn_sidebar_menu.setIcon(QIcon("./img/menu.png"))
        self.btn_sidebar_sensor.setIcon(QIcon("./img/minimize.png"))
        self.btn_sidebar_test.setIcon(QIcon("./img/minimize.png"))

        self.btn_sidebar_menu_2.setIcon(QIcon("./img/menu.png"))
        self.btn_sidebar_sensor_2.setIcon(QIcon("./img/minimize.png"))
        self.btn_sidebar_test_2.setIcon(QIcon("./img/minimize.png"))


        self.btn_sidebar_menu.setStyleSheet(style_btn)
        self.btn_sidebar_menu_2.setStyleSheet(style_btn)

        self.btn_sidebar_sensor.setStyleSheet(style_btn)
        self.btn_sidebar_sensor_2.setStyleSheet(style_btn)

        self.btn_sidebar_test.setStyleSheet(style_btn)
        self.btn_sidebar_test_2.setStyleSheet(style_btn)
        
 
if __name__ == "__main__":
    mainApp = QApplication(sys.argv)
    app = Main_UI()
    app.show()

    try:
        sys.exit(mainApp.exec_())
    except:
        print('Exiting')