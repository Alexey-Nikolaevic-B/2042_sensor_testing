from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
import os

from PyQt5.QtCore import pyqtSignal

icon_path = "./icon"

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
import os

from PyQt5.QtCore import pyqtSignal

icon_path = "./icon"

class TestItem(QWidget):

    def __init__(self, queue):
        super().__init__()
        ui_path = os.path.join(os.path.dirname(__file__), 'qt', 'test_item.ui')
        uic.loadUi(ui_path, self)

        self.queue = queue
        
        self.is_running = False
        self.test_name = None
        self.test_status = "Failed"
        self.test_descripition = ""
        self.progress_value = 0

        self.setup_style()
        self.setup_connections()
    
    def setup_connections(self):
        self.btn_run_stop.clicked.connect(self.toggle_run_stop)

    def set_test_name(self, test_name):
        self.test_name = test_name

    def set_test_status(self, test_status):
        self.test_status = test_status

    def set_progress_value(self, progress_value):
        self.progress_value = progress_value

    def set_test_result(self, test_result):
        self.test_result = test_result

    def set_test_descripition(self, test_descripition):
        self.test_descripition = test_descripition

    def toggle_run_stop(self):
        self.is_running = not self.is_running
        
        if self.is_running:
            self.queue.add_to_queue(self)
        else:
            self.queue.remove_from_queue(self)

        self.update()

    def update(self):
        self.lbl_test_name.setText(str(self.test_name))

        self.update_run_stop_btn()
        self.update_progress_bar()
        self.update_status_icon()

    def update_run_stop_btn(self):
        if self.is_running:
            icon_run_stop = QIcon("icon/stop.png")
            self.btn_run_stop.setIcon(icon_run_stop)
            self.btn_run_stop.setIconSize(QSize(24, 24))
        else:
            icon_run_stop = QIcon("icon/run.png")
            self.btn_run_stop.setIcon(icon_run_stop)
            self.btn_run_stop.setIconSize(QSize(24, 24))

    def update_progress_bar(self):
        if self.is_running: 
            self.progress_bar.show()
            self.test_result_label.hide()
        else:
            self.progress_bar.hide()
            self.test_result_label.show()
        
    def update_status_icon(self):
        if self.test_status == "Passed":
            self.test_result_label.setText(str(self.test_result))

            icon = QIcon("icon/success.png")
            self.status_icon.setPixmap(icon.pixmap(24, 24))
        elif self.test_status == "Failed":
            icon = QIcon("icon/fail.png")
            self.status_icon.setPixmap(icon.pixmap(24, 24))
        else:
            icon = QIcon("icon/pending.png")
            self.status_icon.setPixmap(icon.pixmap(24, 24))
        
    def setup_style(self):
        main_style = """
            QWidget#Test_Item > QWidget {
                background-color: transparent;
            }
        """
        
        self.setStyleSheet(main_style)

        button_style = """
            QPushButton {
                background-color: transparent;
                border: none;
            }
            QPushButton:hover {
                background-color: rgba(255, 255, 255, 0.15);
                border-radius: 5px;
            }
            QPushButton:pressed {
                background-color: rgba(255, 255, 255, 0.25);
            }
        """
        self.btn_run_stop.setStyleSheet(button_style)
        
        # Label style
        label_style = """
            QLabel {
                color: rgb(200, 200, 200);
                background-color: transparent;
            }
            QLabel#lbl_test_name {
                font-weight: bold;
                font-size: 14px;
            }
            QLabel#test_result_label {
                font-size: 12px;
                color: rgb(150, 150, 150);
            }
        """
        self.lbl_test_name.setStyleSheet(label_style)
        self.test_result_label.setStyleSheet(label_style)
        
        # Status icon style
        self.status_icon.setStyleSheet("background-color: transparent;")
        
        progress_style = """
            QProgressBar {
                border: none;
                background-color: rgba(255, 255, 255, 0.1);
                max-height: 10px;
            }
            QProgressBar::chunk {
                background-color: #4CAF50;
            }
        """
        self.progress_bar.setStyleSheet(progress_style)