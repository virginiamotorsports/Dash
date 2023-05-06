from cmath import pi
from ctypes import alignment
from re import S
import os
from unittest import case
from rclpy.clock import Clock
from time import time as now
from math import ceil, isnan

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QGridLayout
from PyQt5.QtCore import Qt, QRunnable, QThread, QThreadPool, pyqtSignal
import PyQt5.QtGui as QtGui
from pydash.rpm import RPMGauge
from PyQt5.QtCore import QTimer
from ament_index_python.packages import get_package_share_directory
image_folder = os.path.join(get_package_share_directory('pydash'), "images")

class Gui():
    def __init__(self, args=[]):
        self.running = True
        self.b = False
        self.root = QApplication(args)
        self.window = QWidget()
        self.window.setWindowTitle("Dashboard")
        self.window.setWindowFlag(Qt.FramelessWindowHint)
        self.init_gui()
        self.windowIcon = QtGui.QIcon()
        self.window.setWindowIcon(self.windowIcon)
        self.window.setStyleSheet("background-color: white;")
        self.window.showMaximized()
        self.window.show()

        timeInterval = 50 # ms
        self.timer = QTimer(self.root, timeout=self.update_widgets, interval=timeInterval)
        self.timer.start()

        self.track_state = 0
        self.vehicle_state = 0
        self.car_position = (0,0)

    def init_gui(self):
        self.layout = QGridLayout()
        self.window.setLayout(self.layout)
        
        self.window.addWidget()


    def run(self):
        sys.exit(self.root.exec())

    def destroy(self):
        self.root.quit()
        self.running = False
    
    def receive_msg(self, topic_name, data):
        switch = {"dash_report": self.data_callback}
        switch[topic_name](data)

    def data_callback(self, data):
        pass

    def update_widgets(self):
        pass