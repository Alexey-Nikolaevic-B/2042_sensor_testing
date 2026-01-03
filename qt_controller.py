from PyQt5 import QtWidgets
from PyQt5.QtCore import QObject, pyqtSignal

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from PyQt5.uic import loadUi

import ui_main 

class QT_Controler(QObject):
    
    def __init__(self):
        QObject.__init__(self)

        self.widget = QtWidgets.QStackedWidget()

        self.main = ui_main.MainScreen()

        self.run()

    def run(self):
        self.widget.addWidget(self.main)
        self.widget.show()