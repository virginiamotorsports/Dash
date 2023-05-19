import sys
from PyQt5.QtCore import Qt, QTimer, QPoint
from PyQt5.QtGui import QPainter, QBrush, QPen, QColor, QPolygon, QFont
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QGridLayout


class InfoClass(QWidget):
      
    def __init__(self, name, data = ""):
        super().__init__()
        self.layout = QGridLayout()
        self.setLayout(self.layout)
        self.infoText = QLabel(name)
        self.infoText.setFont(QFont('Arial', 50))
        self.infoData = QLabel(data)
        self.infoData.setFont(QFont('Arial', 50))
        #self.infoText.setStyleSheet("border: 1px solid black;")
        self.infoData.setStyleSheet("color : red;")
        self.layout.addWidget(self.infoText, 0, 0)
        self.layout.addWidget(self.infoData, 0, 1)
        self.data = data

    def setName(self, name):
        self.infoText.setText(name)
        
    def setData(self, data):
        self.data = str(data)

    def update(self, color = "red"):
        self.infoData.setStyleSheet("color : " + color + ";")
        self.infoData.setText(self.data)
        #self.infoData.setStyleSheet("color : black;")


class SkidGauge(QWidget):
    def __init__(self):
        super().__init__()
        layout = QGridLayout()
        self.setStyleSheet("background-color: white;")

        self.rpm = InfoClass("RPM  ")
        self.throttle = InfoClass("Thr %  ")
        self.gear_disp = QLabel("")
        self.gear_disp.setFont(QFont('Arial', 80))

        layout.addWidget(self.rpm, 0, 0, Qt.AlignLeft)
        layout.addWidget(self.throttle, 1, 0, Qt.AlignLeft)
        layout.addWidget(self.gear, 0, 1, 2, 1, Qt.AlignCenter)
        
        self.setLayout(layout)


    def update_widget(self, gear):
        self.rpm.update()
        self.throttle.update()
        if gear == 0:
            gear = ""
        else:
            gear = str(gear)
        self.gear_disp.setText(gear)