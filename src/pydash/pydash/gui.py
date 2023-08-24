from cmath import pi
from ctypes import alignment
from re import S
import os
from unittest import case
from rclpy.clock import Clock
from time import time as now
from math import ceil, isnan

# import RPi.GPIO as GPIO

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QGridLayout, QStackedWidget
from PyQt5.QtCore import Qt, QRunnable, QThread, QThreadPool, pyqtSignal
import PyQt5.QtGui as QtGui
from dash_msgs.msg import DashReport
from pydash.rpm import RPMGauge
from pydash.brake_screen import BrakeGauge
from pydash.skid_screen import SkidGauge
from math import trunc
from pydash.debug_screen import Debug_Screen
from PyQt5.QtCore import QTimer
from ament_index_python.packages import get_package_share_directory
image_folder = os.path.join(get_package_share_directory('pydash'), "images")

class Gui():
    def __init__(self, args=[]):
        
        # GPIO.setmode(GPIO.BCM) # BCM pin 22
        # self.button_pin = 17
        
        # GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # GPIO.add_event_detect(self.button_pin, GPIO.RISING, callback=self.increment_screen, bouncetime=500)
        
        self.running = True
        self.b = False
        self.num_windows = 0
        self.dash_msg = DashReport()
        self.root = QApplication(args)
        self.window = QStackedWidget()
        self.window.setFixedWidth(640)
        self.window.setFixedHeight(480)
        flags = Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint | Qt.X11BypassWindowManagerHint
        self.window.setWindowFlags(flags)
        self.init_gui()
        # self.windowIcon = QtGui.QIcon()
        # self.window.setWindowIcon(self.windowIcon)
        self.window.setStyleSheet("background-color: white;")
        # self.window.showMinimized()
        self.window.showMaximized()
        # self.window.show()

        timeInterval = 50 # ms
        self.timer = QTimer(self.root, timeout=self.update_widgets, interval=timeInterval)
        self.timer.start()
        
        # self.timer2 = QTimer()
        # self.timer2.timeout.connect(self.setup_pins)
        # self.timer2.start(1000)

        self.track_state = 0
        self.vehicle_state = 0
        self.car_position = (0,0)
        
        

    def init_gui(self):        
        self.rpm_g = RPMGauge()
        self.window.addWidget(self.rpm_g)
        self.num_windows+=1
        
        self.debug = Debug_Screen()
        self.window.addWidget(self.debug)
        self.num_windows+=1
        
        self.brake = BrakeGauge()
        self.window.addWidget(self.brake)
        self.num_windows+=1
        
        self.skid = SkidGauge()
        self.window.addWidget(self.skid)
        self.num_windows+=1
        
        self.window.setCurrentIndex(1)
        
    # def reShow(self):
    #     self.window.showMinimized()
    #     self.window.setWindowState(self.window.windowState() and (not Qt.WindowMinimized or Qt.WindowActive))

    # def increment_screen(self, arg1):
    #     if self.window.currentIndex() == self.num_windows - 1:
    #         self.window.setCurrentIndex(0)
    #     else:
    #         self.window.setCurrentIndex(self.window.currentIndex() + 1)

    def run(self):
        sys.exit(self.root.exec())

    def destroy(self):
        self.root.quit()
        self.running = False
        
    # def setup_pins(self):
    #     try:
            
            
    #         self.timer2.stop()
    #     except:
    #         print("Failed to get gpio pins")
    
    def receive_msg(self, topic_name, data):
        switch = {"dash_report": self.data_callback}
        switch[topic_name](data)

    def data_callback(self, data):
        self.dash_msg = data
        # print(data)

    def update_widgets(self):
        if self.window.currentIndex() == 0:
            self.rpm_g.update_rpm(trunc(self.dash_msg.engine_rpm), self.dash_msg.gear)
        elif self.window.currentIndex() == 1:
            self.debug.fuel.setData(round(self.dash_msg.fuel_pressure, 1))
            self.debug.engine_temp.setData(round(self.dash_msg.coolant_temp, 1))
            self.debug.rpm.setData(round(self.dash_msg.engine_rpm, -2))
            self.debug.oil_pres.setData(round(self.dash_msg.oil_pressure, 1))
            self.debug.oil_temp.setData(round(self.dash_msg.oil_temp, 1))
            self.debug.throttle.setData(round(self.dash_msg.throttle_pos, 1))
            self.debug.update_widget()
        elif self.window.currentIndex() == 2:
            self.brake.throttle.setData(round(self.dash_msg.throttle_pos, 1))
            self.brake.rpm.setData(round(self.dash_msg.engine_rpm, -2))
            self.brake.update_widget()
        elif self.window.currentIndex() == 3:
            self.skid.throttle.setData(round(self.dash_msg.throttle_pos, 1))
            self.skid.rpm.setData(round(self.dash_msg.engine_rpm, -2))
            self.skid.update_widget(self.dash_msg.gear)
        