import sys
from PyQt5.QtCore import Qt, QTimer, QPoint, QRect
from PyQt5.QtGui import QPainter, QBrush, QPen, QColor, QPolygon, QFont, QPixmap
from PyQt5.QtWidgets import QApplication, QWidget

from ament_index_python.packages import get_package_share_directory

image_folder = map_dir = get_package_share_directory('pydash')


class RPMGauge(QWidget):
    def __init__(self):
        super().__init__()
        self.rpm = 0
        self.gear = "N"
        self.setStyleSheet("background-color: white;")
        self.checkpoint = 1
        self.widths = self.width()
        self.heights = self.height()
        self.min_size = min(self.widths, self.heights)
        self.offset = int(max(self.widths, self.heights) - min(self.widths, self.heights) / 2)
        self.padding = 10
        self.file_path = ("%s/images/tach1.png"%(image_folder))

    def update_rpm(self, rpm_value, gear):
        self.rpm = rpm_value
        if gear == 0:
            self.gear = ""
        else:
            self.gear = str(gear)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        font_size = 12
        rpm_font = painter.font()
        rpm_font.setPointSize(font_size)
        painter.setFont(rpm_font)
        painter.translate(self.widths / 2, self.heights / 2)
        
       
        
        rect1 = QRect(-int(self.min_size/2), -int(self.min_size/2), self.min_size, self.min_size)
        pixmap = QPixmap(self.file_path)
        painter.drawPixmap(rect1, pixmap)
        
         # Draw the gear value
        font = QFont("Arial", 150, 100, False)
        painter.setFont(font)
        painter.drawText(-70, 60, str(self.gear))

        
        # Draw the RPM value
        font = QFont("Arial", 50, 50, False)
        painter.setFont(font)
        painter.drawText(-100, 210, str(self.rpm))
        
        painter.setPen(QPen(Qt.red, 6))
        # painter.drawLine(10, 10, 100, 100)

        # Draw the needle
        painter.setBrush(QBrush(QColor(255, 0, 0)))
        # painter.setPen(QPen(Qt.NoPen))
        painter.rotate(-125 + (self.rpm / 11000.0) * 275.0)
        # painter.drawConvexPolygon(QPolygon([
        #     QPoint(0, 0),
        #     QPoint(10, 5),
        #     QPoint(0, -self.min_size // 2 + self.padding + 20),
        #     QPoint(10, 5),
        # ]))
        painter.drawLine(0, -125, 0, -200)