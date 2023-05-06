import sys
from PyQt5.QtCore import Qt, QTimer, QPoint
from PyQt5.QtGui import QPainter, QBrush, QPen, QColor, QPolygon, QFont
from PyQt5.QtWidgets import QApplication, QWidget


class RPMGauge(QWidget):
    def __init__(self):
        super().__init__()
        self.rpm = 0
        self.setWindowTitle("RPM")
        self.setGeometry(100, 100, 300, 300)
        self.setFixedSize(600, 600)
        self.setStyleSheet("background-color: white;")
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_rpm)
        self.timer.start(1)  # update every ms
        self.checkpoint = 1

    def update_rpm(self):
        # Replace this with RPM code
        if self.rpm % (self.checkpoint * 2000) == 0 and self.rpm != 0:
            self.rpm -= 300
            self.checkpoint += 1
        elif self.rpm >= 10000:
            self.rpm = 0
            self.checkpoint = 1
        else:
            self.rpm += 2
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        width = self.width()
        height = self.height()
        min_size = min(width, height)
        padding = 10
        font_size = 12
        rpm_font = painter.font()
        rpm_font.setPointSize(font_size)
        painter.setFont(rpm_font)

        # Draw the outer circle
        painter.setPen(QPen(Qt.black, 2, Qt.SolidLine))
        painter.drawEllipse(padding, padding, min_size - padding * 2, min_size - padding * 2)

        # Draw the ticks and labels
        painter.setPen(QPen(Qt.black, 3, Qt.SolidLine))
        font = QFont("Arial", 18, 60, False)
        painter.setFont(font)
        painter.translate(width / 2, height / 2)

        painter.rotate(210)  # position of 0 rpm

        for i in range(51):
            if i % 5 == 0:
                painter.drawLine(0, -min_size // 2 + padding + 5, 0, -min_size // 2 + padding + 40)

                painter.drawText(-20, -min_size // 2 + padding + 55, str(i // 5 * 1000))
            else:
                painter.drawLine(0, -min_size // 2 + padding + 5, 0, -min_size // 2 + padding + 25)

            painter.rotate(6)

        painter.rotate(204)

        # Draw the RPM value
        painter.drawText(-20, 50, str(self.rpm))

        # Draw the needle
        painter.setBrush(QBrush(QColor(255, 0, 0)))
        painter.setPen(QPen(Qt.NoPen))
        painter.rotate(-150 + (self.rpm / 10000.0) * 300.0)
        painter.drawConvexPolygon(QPolygon([
            QPoint(0, 0),
            QPoint(10, -5),
            QPoint(0, -min_size // 2 + padding + 20),
            QPoint(-10, -5),
        ]))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    widget = RPMGauge()
    widget.show()
    sys.exit(app.exec_())

