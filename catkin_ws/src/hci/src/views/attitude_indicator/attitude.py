import sys
from PyQt5.QtCore import Qt
from math import sqrt, atan, degrees

from PyQt5.QtCore import QPointF
from PyQt5.QtCore import QRectF
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtGui import QBrush
from PyQt5.QtGui import QColor
from PyQt5.QtGui import QFont
from PyQt5.QtGui import QPainter
from PyQt5.QtGui import QPen
from PyQt5.QtGui import QPolygon
from PyQt5.QtGui import QRegion
from PyQt5.QtGui import QTextOption
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QWidget



class QAttitude(QWidget):
    canvasReplot = pyqtSignal(name="canvasReplot")

    def __init__(self, parent = None):
        super(QAttitude, self).__init__(parent)

        self.m_sizeMin = 200
        self.m_sizeMax = 600
        self.m_offset = 2
        self.m_size = self.m_sizeMin - 2*self.m_offset
        self.m_roll = 0.0
        self.m_pitch = 0.0

        self.setMinimumSize(self.m_sizeMin, self.m_sizeMin)
        self.setMaximumSize(self.m_sizeMax, self.m_sizeMax)
        self.resize(self.m_sizeMin, self.m_sizeMin)

        self.canvasReplot.connect(self.canvasReplot_slot)

    def setData(self, p, r):
        self.setPitch(p)
        self.setRoll(r)

    def setPitch(self, val):
        self.m_pitch = val
        self.m_pitch = max(self.m_pitch, -90)
        self.m_pitch = min(self.m_pitch, 90)

    def setRoll(self, val):
        self.m_roll = val
        self.m_roll = max(self.m_roll, -180)
        self.m_roll = min(self.m_roll, 180)

    def getPitch(self):
        return self.m_pitch

    def getRoll(self):
        return self.m_roll

    @pyqtSlot(name="canvasReplot_slot")
    def canvasReplot_slot(self):
        self.update()

    def paintEvent(self, QPaintEvent):
        painter = QPainter(self)
        bgSky = QBrush(QColor(48,172,220))
        bgGround = QBrush(QColor(247,168,21))

        whitePen = QPen(Qt.white)
        blackPen = QPen(Qt.black)
        pitchPen = QPen(Qt.white)
        pitchZero = QPen(Qt.green)

        whitePen.setWidth(2)
        blackPen.setWidth(2)
        pitchZero.setWidth(3)

        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(self.width() / 2, self.height() / 2)
        painter.rotate(self.m_roll)

        # FIXME: AHRS output left-hand values
        pitch_tem = -self.m_pitch

        # draw background
        y_min = self.m_size / 2 * -40.0 / 45.0
        y_max = self.m_size / 2 * 40.0 / 45.0

        y = self.m_size / 2 * pitch_tem / 45.0
        y = min(y, y_min)
        y = max(y, y_max)

        x = sqrt(self.m_size * self.m_size / 4 - y * y)
        gr = atan(y / x)

        painter.setPen(blackPen)
        painter.setBrush(bgSky)
        painter.drawChord(-self.m_size / 2, -self.m_size / 2, self.m_size, self.m_size, gr * 16, (180 - 2 * gr) * 16)
        painter.setBrush(bgGround)
        painter.drawChord(-self.m_size / 2, -self.m_size / 2, self.m_size, self.m_size, gr * 16, -(180 + 2 * gr) * 16)

        maskRegion = QRegion(-self.m_size / 2, -self.m_size / 2, self.m_size, self.m_size, QRegion.Ellipse)
        painter.setClipRegion(maskRegion)

        # draw pitch lines & marker
        ll = self.m_size / 8
        fontSize = 8

        pitchPen.setWidth(2)
        painter.setFont(QFont("", fontSize))

        for i in range(-9, 9, 1):
            p = i * 10

            s = str(-p)
            if i % 3 == 0:
                l = ll
            else:
                l = ll / 2

            if i == 0:
                painter.setPen(pitchZero)
                l *= 1.8
            else:
                painter.setPen(pitchPen)

            y = self.m_size / 2 * p / 45.0 - self.m_size / 2 * pitch_tem / 45.0
            x = l

            r = sqrt(x * x + y * y)
            if r > self.m_size / 2:
                continue

            painter.drawLine(QPointF(-l, 1.0 * y), QPointF(l, 1.0 * y))

            textWidth = 100

            if i % 3 == 0 and i != 0:
                painter.setPen(whitePen)

                x1 = -x - 2 - textWidth
                y1 = y - fontSize / 2 - 1
                painter.drawText(QRectF(x1, y1, textWidth, fontSize + 2), Qt.AlignRight | Qt.AlignVCenter, s)

        # draw markers

        markerSize = self.m_size / 20
        painter.setBrush(Qt.red)
        painter.setPen(Qt.NoPen)

        fx1 = markerSize
        fy1 = 0
        fx2 = fx1 + markerSize
        fy2 = -markerSize / 2
        fx3 = fx1 + markerSize
        fy3 = markerSize / 2

        points = (QPointF(fx1, fy1), QPointF(fx2, fy2), QPointF(fx3, fy3))
        poly = QPolygon()
        for po in points:
            poly.append(po.toPoint())
        painter.drawPolygon(poly)

        points2 = [QPointF(-fx1, fy1), QPointF(-fx2, fy2), QPointF(-fx3, fy3)]
        poly2 = QPolygon()
        for po in points2:
            poly2.append(po.toPoint())
        painter.drawPolygon(poly2)

        # draw roll degree lines
        nRollLines = 36
        rotAng = 360.0 / nRollLines
        rollLineLeng = self.m_size / 25
        blackPen.setWidth(1)
        painter.setPen(blackPen)
        painter.setFont(QFont("", fontSize))

        for i in range(0, nRollLines):
            if (i < nRollLines / 2):
                s = str(-i * rotAng)
            else:
                s = str(360 - i * rotAng)

            fx1 = 0
            fy1 = -self.m_size / 2 + self.m_offset
            fx2 = 0

            if (i % 3 == 0):
                fy2 = fy1 + rollLineLeng
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2))

                fy2 = fy1 + rollLineLeng + 2
                painter.drawText(QRectF(-50, fy2, 100, fontSize + 2), Qt.AlignCenter, s)

            else:
                fy2 = fy1 + rollLineLeng / 2
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2))

            painter.rotate(rotAng)

        # draw roll marker
        rollMarkerSize = self.m_size / 25
        painter.rotate(-self.m_roll)
        painter.setBrush(QBrush(Qt.black))

        fx1 = 0
        fy1 = -self.m_size / 2 + self.m_offset
        fx2 = fx1 - rollMarkerSize / 2
        fy2 = fy1 + rollMarkerSize
        fx3 = fx1 + rollMarkerSize / 2
        fy3 = fy1 + rollMarkerSize

        points = [QPointF(fx1, fy1),QPointF(fx2, fy2),QPointF(fx3, fy3)]
        poly = QPolygon()
        for po in points:
            poly.append(po.toPoint())
        painter.drawPolygon(poly)

    def resizeEvent(self, QResizeEvent):
        self.m_size = min(self.width(), self.height()) - 2 * self.m_offset

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = QAttitude()
    win.setData(0, 0)
    win.show()
    exit(app.exec())