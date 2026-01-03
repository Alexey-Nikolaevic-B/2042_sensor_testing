from PyQt5.QtWidgets import QDialog
from PyQt5.QtCore import pyqtSignal
from PyQt5.uic import loadUi
from PyQt5 import QtCore

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

class MainScreen(QDialog):

    def __init__(self):
        super(MainScreen, self).__init__()
        self.init_ui()

    def init_ui(self):
        loadUi(('./qt/main.ui'), self)