import sys
from PyQt5.QtCore import Qt, QTimer, QPoint
from PyQt5.QtGui import QPainter, QBrush, QPen, QColor, QPolygon, QFont
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QGridLayout


class InfoClass2(QWidget):
      
    def __init__(self, name, data = ""):
        super().__init__()
        self.layout = QGridLayout()
        self.setLayout(self.layout)
        self.infoText = QLabel(name)
        self.infoText.setFont(QFont('Arial', 60))
        self.infoData = QLabel(data)
        self.infoData.setFont(QFont('Arial', 60))
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


class BrakeGauge(QWidget):
    def __init__(self):
        super().__init__()
        layout = QGridLayout()
        self.setStyleSheet("background-color: white;")

        self.rpm = InfoClass2("RPM  ")
        self.throttle = InfoClass2("Thr %  ")

        layout.addWidget(self.rpm, 0, 0, Qt.AlignLeft)
        layout.addWidget(self.throttle, 1, 0, Qt.AlignLeft)
        
        self.setLayout(layout)


    def update_widget(self):
        self.rpm.update()
        self.throttle.update()