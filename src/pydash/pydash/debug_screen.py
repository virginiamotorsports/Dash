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
        self.infoText.setFont(QFont('Arial', 32))
        self.infoData = QLabel(data)
        self.infoData.setFont(QFont('Arial', 32))
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


class Debug_Screen(QWidget):
    def __init__(self):
        super().__init__()
        layout = QGridLayout()
        self.setStyleSheet("background-color: white;")
        self.engine_temp = InfoClass("Eng T")
        self.oil_temp = InfoClass("Oil T")
        self.oil_pres = InfoClass("Oil P")
        self.rpm = InfoClass("RPM")
        self.throttle = InfoClass("Thr %")
        self.fuel = InfoClass("Fuel P")
        
        layout.addWidget(self.engine_temp, 0, 0, Qt.AlignCenter)
        layout.addWidget(self.oil_temp, 0, 1, Qt.AlignCenter)
        layout.addWidget(self.oil_pres, 1, 0, Qt.AlignCenter)
        layout.addWidget(self.rpm, 1, 1, Qt.AlignCenter)
        layout.addWidget(self.throttle, 2, 0, Qt.AlignCenter)
        layout.addWidget(self.fuel, 2, 1, Qt.AlignCenter)
        
        self.setLayout(layout)


    def update_widget(self):
        self.engine_temp.update()
        self.oil_temp.update()
        self.oil_pres.update()
        self.rpm.update()
        self.throttle.update()
        self.fuel.update()