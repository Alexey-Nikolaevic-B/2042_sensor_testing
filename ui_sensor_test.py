# testing_window.py
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic
import numpy as np
from datetime import datetime

import json

from queue_manager import QueueManager
from ui_test_item import TestItem

icon_path = "./icon"

class TestingWindow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi('./qt/sensor_test.ui', self)
        
        self.queue = QueueManager() 

        self.sensor_data = None
        self.is_streaming = False
        self.test_results = {}
        self.current_test = None
        
        self.setup_styles()
        self.setup_connections()

    def setup_connections(self):
        self.list_tests.itemSelectionChanged.connect(self.test_selected)

    def test_selected(self):
        selected_items = self.list_tests.selectedItems()
        if selected_items:
            item = selected_items[0]
            test = self.list_tests.itemWidget(item)

            self.label_test_description.setText(test.test_descripition)

    def load_new_sensor(self, sensor_data):
        self.sensor_data = sensor_data

        # self.lbl_sensor_name.setText(self.sensor_data['name'])

        self.load_tests_into_list()


    def load_tests_into_list(self):
        self.list_tests.clear()
        
        test_count = 0
        for test_data in self.sensor_data['tests']:
            test_count += 1
            list_item = QListWidgetItem(self.list_tests)
            list_item.setSizeHint(QSize(400, 75))
            
            test_widget = TestItem(self.queue)
            test_widget.set_test_name(test_data['name'])
            test_widget.set_test_result(0)
            test_widget.set_test_status(test_data['status'])
            test_widget.set_test_descripition(test_data['description'])
            test_widget.set_progress_value(0)
            test_widget.update()
            
            self.list_tests.addItem(list_item)
            self.list_tests.setItemWidget(list_item, test_widget)
            test_widget.list_item = list_item
        
        # Calculate height based on number of items
        item_height = 75
        spacing = self.list_tests.spacing() or 0
        frame_margin = self.list_tests.frameWidth() * 2
        
        # Calculate total height for all items
        total_height = (test_count * (item_height + spacing)) + frame_margin
        max_height = min(400, total_height)
        
        # Set size policy and fixed height
        self.list_tests.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.list_tests.setFixedHeight(max_height)
        
        # Ensure the scroll area propagates the size correctly
        self.scrollArea_tests.setWidgetResizable(True)
        self.scrollArea_tests.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # Set alignment of the container layout to top
        container_layout = self.scrollAreaWidgetContents_tests.layout()
        container_layout.setAlignment(Qt.AlignTop)

    def setup_styles(self):
        self.setStyleSheet("""
            /* ===== BASE WIDGET STYLES ===== */
            /* Remove focus outlines from all widgets */
            *:focus {
                outline: none;
            }
            
            /* ===== PUSH BUTTON STYLES ===== */
            /* Base button styles */
            QPushButton {
                border-radius: 4px;
                background-color: transparent;
                color: #cccccc;
                outline: none;
            }
            
            QPushButton:hover {
                background-color: #3a3a3a;
            }
            
            QPushButton:pressed {
                background-color: #252525;
            }
            
            /* Specific button styles - Export */
            QPushButton#btn_export {
                background-color: #25394d;
                border-color: #3498db;
                color: #5dade2;
            }
            
            QPushButton#btn_export:hover {
                background-color: #2c3e50;
            }
            
            /* Specific button styles - New Test */
            QPushButton#btn_new_test {
                background-color: #1e3a2a;
                border-color: #2ecc71;
                color: #27ae60;
            }
            
            QPushButton#btn_new_test:hover {
                background-color: #225633;
            }
            
            /* Specific button styles - Close Details */
            QPushButton#btn_close_details {
                background-color: #2d2d2d;
                border: 1px solid #444444;
                color: #aaaaaa;
                font-weight: bold;
            }
            
            QPushButton#btn_close_details:hover {
                background-color: #3a3a3a;
                color: #cccccc;
            }
            
            /* ===== GROUP BOX STYLES ===== */
            /* Base group box styles */
            QGroupBox {
                border: 1px solid #333333;
                border-radius: 4px;
                font-weight: bold;
                color: #d3d3d3;
            }
            
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
            }
            
            /* Specific group boxes */
            QGroupBox#groupBox_test_description,
            QGroupBox#groupBox_logs {
                color: #d3d3d3;
                border: 1px solid #3a3a3a;
            }
            
            QGroupBox#groupBox_test_description::title,
            QGroupBox#groupBox_logs::title {
                color: #e0e0e0;
            }
            
            /* ===== LABEL STYLES ===== */
            /* Base label styles */
            QLabel {
                color: #e0e0e0;
            }
            
            /* Labels with black backgrounds */
            QLabel#sensor_view_label,
            QLabel#sensor_image,
            QLabel#test_image,
            QLabel#label_log {
                background-color: #000000;
                color: #ffffff;
                font-weight: bold;
            }
            
            /* Labels with dark gray backgrounds */
            QLabel#image_label,
            QLabel#horizontalLayout {
                background-color: #252525;
                border: 1px solid #333333;
            }
            
            /* Sensor info labels */
            QLabel#sensor_name {
                font-size: 20px;
                font-weight: bold;
                color: white;
            }
            
            QLabel#type_label, 
            QLabel#id_label, 
            QLabel#status_label,
            QLabel#description_label {
                font-weight: bold;
                color: #dddddd;
            }
            
            QLabel#status_label {
                color: #27ae60;
            }
            
            QLabel#description_label {
                color: #aaaaaa;
            }
            
            /* Secondary text */
            QLabel[style*="secondary"] {
                color: #b0b0b0;
            }
            
            /* Test description label */
            QLabel#label_test_description {
                background-color: transparent;
            }
            
            /* ===== FRAME STYLES ===== */
            /* Frames with dark backgrounds */
            QFrame#test_bar,
            QFrame#frame_test_description,
            QFrame#frame_sensor_view_2 {
                background-color: #252525;
            }
            
            /* Remove outline from containers */
            QGroupBox, 
            QFrame, 
            QWidget#container, 
            QWidget#main_container, 
            QWidget#content {
                outline: none;
                border: none;
            }
            
            /* ===== LIST WIDGET STYLES ===== */
            QListWidget {
                border: 1px solid #333333;
                background-color: #252525;
                alternate-background-color: #2a2a2a;
                outline: none;
            }
            
            QListWidget::item {
                border-bottom: 1px solid #333333;
                color: #cccccc;
            }
            
            QListWidget::item:selected {
                background-color: #3a3a3a;
                color: white;
            }
            
            QListWidget::item:hover {
                background-color: #2d2d2d;
            }
            
            QListWidget::item:selected:hover {
                background-color: #454545;
            }
            
            QListWidget::item:focus {
                outline: none;
                border: none;
            }
            
            /* ===== SCROLL BAR STYLES ===== */
            QScrollBar:vertical {
                background: transparent;
                width: 0px;
            }
            
            QScrollBar::handle:vertical {
                background: transparent;
            }
            
            QScrollBar::add-line:vertical, 
            QScrollBar::sub-line:vertical {
                height: 0px;
                background: transparent;
            }
            
            QScrollBar::add-page:vertical, 
            QScrollBar::sub-page:vertical {
                background: transparent;
            }
        """)

        icon = QIcon("icon/run_all.png")
        self.btn_run_all.setIcon(icon)
        self.btn_run_all.setIconSize(QSize(24, 24))