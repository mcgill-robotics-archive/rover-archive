import sys
from PyQt5.QtCore import QPointF
from PyQt5.QtCore import QRectF
from PyQt5.QtCore import Qt
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QBrush
from PyQt5.QtGui import QColor
from PyQt5.QtGui import QFont
from PyQt5.QtGui import QPainter
from PyQt5.QtGui import QPen
from PyQt5.QtGui import QPolygon
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QWidget


class QCompass(QWidget):
    canvasReplot = pyqtSignal(name="canvasReplot")

    def __init__(self, parent=None):
        super(QWidget, self).__init__(parent)

        self.m_sizeMin = 200
        self.m_sizeMax = 600
        self.m_offset = 2
        self.m_size = self.m_sizeMin - 2 * self.m_offset
        self.m_yaw = 0.0
        self.m_alt = 0.0
        self.m_h = 0.0

        self.setMinimumSize(self.m_sizeMin, self.m_sizeMin)
        self.setMaximumSize(self.m_sizeMax, self.m_sizeMax)
        self.resize(self.m_sizeMin, self.m_sizeMin)
        self.setFocusPolicy(Qt.NoFocus)
        self.canvasReplot.connect(self.canvasReplot_slot)

    def setData(self, y, a, h):
        self.setYaw(y)
        self.setH(h)
        self.setAlt(a)

    def setYaw(self, val):
        self.m_yaw = val
        self.m_yaw = min(self.m_yaw, 360)
        self.m_yaw = max(self.m_yaw, -360)

    def setAlt(self, val):
        self.m_alt = val

    def setH(self, val):
        self.m_h = val


    def getYaw(self):
        return self.m_yaw

    def getAlt(self):
        return self.m_alt

    def getH(self):
        return self.m_h

    def canvasReplot_slot(self):
        self.update()

    def paintEvent(self, QPaintEvent):
        painter = QPainter(self)
        bgGround = QBrush(QColor(48, 172, 220))

        whitePen = QPen(Qt.white)
        blackPen = QPen(Qt.black)
        greenPen = QPen(Qt.green)
        redPen = QPen(Qt.red)
        bluePen = QPen(Qt.blue)

        whitePen.setWidth(1)
        blackPen.setWidth(2)
        greenPen.setWidth(2)
        redPen.setWidth(2)
        bluePen.setWidth(2)

        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(self.width() / 2, self.height() / 2)

        # draw background

        painter.setPen(blackPen)
        painter.setBrush(bgGround)

        painter.drawEllipse(-self.m_size / 2, -self.m_size / 2,self. m_size, self.m_size)

        # draw yaw lines

        nyawLines = 36
        rotAng = 360.0 / nyawLines
        yawLineLeng = self.m_size / 25
        fontSize = 8

        blackPen.setWidth(1)
        painter.setPen(blackPen)

        s = ""

        for i in range(0, nyawLines):
            if i == 0:
                s = "N"
                painter.setPen(bluePen)
                painter.setFont(QFont("", fontSize * 1.3))

            elif i == 9:
                s = "W"
                painter.setPen(blackPen)
                painter.setFont(QFont("", fontSize * 1.3))
            
            elif i == 18:
                s = "S"
                painter.setPen(redPen)
                painter.setFont(QFont("", fontSize * 1.3))
            elif i == 27:
                s = "E"
                painter.setPen(blackPen)
                painter.setFont(QFont("", fontSize * 1.3))
            else:
                s = str(i * rotAng)
                painter.setPen(blackPen)
                painter.setFont(QFont("", fontSize))

            fx1 = 0
            fy1 = -self.m_size / 2 + self.m_offset
            fx2 = 0

            if i % 3 == 0:
                fy2 = fy1 + yawLineLeng
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2))

                fy2 = fy1 + yawLineLeng+4
                painter.drawText(QRectF(-50, fy2, 100, fontSize+2), Qt.AlignCenter, s)
            else :
                fy2 = fy1 + yawLineLeng / 2
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2))

            painter.rotate(-rotAng)

        #  draw S/N arrow
        arrowWidth = self.m_size / 5

        fx1 = 0
        fy1 = -self.m_size / 2 + self.m_offset + self.m_size / 25 + 15
        fx2 = -arrowWidth / 2
        fy2 = 0
        fx3 = arrowWidth / 2
        fy3 = 0

        painter.setPen(Qt.NoPen)

        painter.setBrush(QBrush(Qt.blue))

        pointsN = [QPointF(fx1, fy1),QPointF(fx2, fy2),QPointF(fx3, fy3)]
        polyN = QPolygon()
        for point in pointsN:
            polyN.append(point.toPoint())
        painter.drawPolygon(polyN)

        fx1 = 0
        fy1 = self.m_size / 2 - self.m_offset - self.m_size / 25 - 15
        fx2 = -arrowWidth / 2
        fy2 = 0
        fx3 = arrowWidth / 2
        fy3 = 0

        painter.setBrush(QBrush(Qt.red))
        pointsS = [QPointF(fx1, fy1), QPointF(fx2, fy2), QPointF(fx3, fy3)]
        polyS = QPolygon()
        for point in pointsS:
            polyS.append(point.toPoint())
        painter.drawPolygon(polyS)

        # draw yaw marker

        yawMarkerSize = self.m_size / 12

        painter.rotate(-self.m_yaw)  # could possibly move this before drawing NS arrow
        painter.setBrush(QBrush(QColor(0xFF, 0x00, 0x00, 0xE0)))

        fx1 = 0
        fy1 = -self.m_size / 2 + self.m_offset
        fx2 = fx1 - yawMarkerSize / 2
        fy2 = fy1 + yawMarkerSize
        fx3 = fx1 + yawMarkerSize / 2
        fy3 = fy1 + yawMarkerSize

        pointsYaw = [QPointF(fx1, fy1), QPointF(fx2, fy2), QPointF(fx3, fy3)]
        polyYaw = QPolygon()
        for point in pointsYaw:
            polyYaw.append(point.toPoint())
        painter.drawPolygon(polyYaw)

        painter.rotate(self.m_yaw)

        altFontSize = 13
        w = 130
        h = 2 * (altFontSize + 8)
        fx = -w / 2
        fy = -h / 2

        blackPen.setWidth(2)
        painter.setPen(blackPen)
        painter.setBrush(QBrush(Qt.white))
        painter.setFont(QFont("", altFontSize))

        painter.drawRoundedRect(fx, fy, w, h, 6, 6)
        painter.setPen(bluePen)
        text = "ALT: " + str(self.m_alt) + "m"
        painter.drawText(QRectF(fx, fy+2, w, h/2), Qt.AlignCenter, text)

        text = "H: " + str(self.m_h) + "m"
        painter.drawText(QRectF(fx, fy+h/2, w, h / 2), Qt.AlignCenter, text)

    def resizeEvent(self, QResizeEvent):
        self.m_size = min(self.width(), self.height()) - 2 * self.m_offset


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = QCompass()
    win.setData(0, 0, 0)
    win.show()
    exit(app.exec())