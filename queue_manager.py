import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSignal, QObject

class QueueManager(QObject):
    def __init__(self):
        super().__init__()
        self.queue = []
        
    def add_to_queue(self, test_widget):
        if test_widget not in self.queue:
            self.queue.append(test_widget)
            self.process_queue()
            
    def remove_from_queue(self, test_widget):
        if test_widget in self.queue:
            self.queue.remove(test_widget)
            
    def process_queue(self):
        pass