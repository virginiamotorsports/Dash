import sys
from PyQt5.QtCore import Qt, QTimer, QPoint
from PyQt5.QtGui import QPainter, QBrush, QPen, QColor, QPolygon, QFont
from PyQt5.QtWidgets import QApplication, QWidget


class RPMGauge(QWidget):
    def __init__(self):
        super().__init__()
        self.rpm = 0
        self.setStyleSheet("background-color: white;")
        self.checkpoint = 1
        self.widths = self.width()
        self.heights = self.height()
        self.min_size = min(self.widths, self.heights)
        self.offset = int(max(self.widths, self.heights) - min(self.widths, self.heights) / 2)
        self.padding = 10

    def update_rpm(self, rpm_value):
        self.rpm = rpm_value
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        font_size = 12
        rpm_font = painter.font()
        rpm_font.setPointSize(font_size)
        painter.setFont(rpm_font)

        # Draw the outer circle
        painter.setPen(QPen(Qt.black, 2, Qt.SolidLine))
        painter.drawEllipse(self.offset, 0, self.min_size - self.offset, self.min_size)

        # Draw the ticks and labels
        painter.setPen(QPen(Qt.black, 3, Qt.SolidLine))
        font = QFont("Arial", 20, 60, False)
        painter.setFont(font)
        painter.translate(self.widths / 2, self.heights / 2)
        rpm_marks = [(-200, 150), (-240, 50), (-235, -55), (-185, -150), (-110, -210), (-20, -230),

                     (70, -210), (145, -150), (195, -55), (200, 50), (145, 145), (90, 210)]
        for i in range(len(rpm_marks)):
            painter.drawText(rpm_marks[i][0], rpm_marks[i][1], str(i * 1000))

        painter.rotate(-125)  # position of 0 rpm

        for i in range(56):
            if i % 5 == 0:
                painter.drawLine(0, -self.min_size // 2 + self.offset + 5, 0, -self.min_size // 2 + self.offset + 40)
            else:
                painter.drawLine(0, -self.min_size // 2 + self.offset + 5, 0, -self.min_size // 2 + self.offset + 25)

            painter.rotate(5)

        painter.rotate(205)

        # Draw the RPM value
        font = QFont("Arial", 70, 60, False)
        painter.setFont(font)
        painter.drawText(-110, 210, str(self.rpm))

        # Draw the needle
        painter.setBrush(QBrush(QColor(255, 0, 0)))
        painter.setPen(QPen(Qt.NoPen))
        painter.rotate(-125 + (self.rpm / 11000.0) * 275.0)
        painter.drawConvexPolygon(QPolygon([
            QPoint(0, 0),
            QPoint(10, -5),
            QPoint(0, -self.min_size // 2 + self.padding + 20),
            QPoint(-10, -5),
        ]))