"""!@brief Digital compass widget"""

import sys
from PyQt5.QtCore import QPointF, pyqtSlot
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
    """!@brief Display a heading and altitude using a compass"""
    canvasReplot = pyqtSignal(name="canvasReplot")

    def __init__(self, parent=None):
        """!@brief Constructor initializes member data and sets object properties

        @param self Python object pointer
        @param parent QWidget parent in Qt hierarchy
        """
        super(QWidget, self).__init__(parent)

        self._size_min = 200
        self._size_max = 600
        self._offset = 2
        self._size = self._size_min - 2 * self._offset
        self._yaw = 0.0
        self._alt = 0.0
        self._h = 0.0

        self.setMinimumSize(self._size_min, self._size_min)
        self.setMaximumSize(self._size_max, self._size_max)
        self.resize(self._size_min, self._size_min)
        self.setFocusPolicy(Qt.NoFocus)
        self.canvasReplot.connect(self.canvasReplot_slot)

    @pyqtSlot(float, float, float)
    def setData(self, y, a, h):
        """!@brief Sets all member data values

        @param self Python object pointer
        @param y The new yaw value
        @param a The new altitude values
        @param h The new h values
        """
        self.setYaw(y)
        self.setH(h)
        self.setAlt(a)

    @pyqtSlot(float)
    def setYaw(self, val):
        """!@brief Set a new value for yaw.

        This is a qt slot so it is thread safe when called using the
        signal-slot mechanism.

        @param self Python object pointer
        @param val The new yaw value
        """
        self._yaw = val
        self._yaw = min(self._yaw, 360)
        self._yaw = max(self._yaw, -360)
        self.canvasReplot_slot()

    @pyqtSlot(float)
    def setAlt(self, val):
        """!@brief Set a new value for altitude.

        This is a qt slot so it is thread safe when called using the
        signal-slot mechanism.

        @param self Python object pointer
        @param val The new altitude value
        """
        self._alt = val
        self.canvasReplot_slot()

    @pyqtSlot(float)
    def setH(self, val):
        """!@brief Set a new value for h.

        This is a qt slot so it is thread safe when called using the
        signal-slot mechanism.

        @param self Python object pointer
        @param val The h altitude value
        """
        self._h = val
        self.canvasReplot_slot()

    def getYaw(self):
        return self._yaw

    def getAlt(self):
        return self._alt

    def getH(self):
        return self._h

    @pyqtSlot()
    def canvasReplot_slot(self):
        """!@brief Force a redraw

        @param self Python object pointer
        """
        self.update()

    def paintEvent(self, QPaintEvent):
        """!@Draw the compass

        Paint event called by the Qt event handler

        @param self Python object pointer
        @param QPaintEvent Qt event
        """
        painter = QPainter(self)
        bg_ground = QBrush(QColor(48, 172, 220))

        white_pen = QPen(Qt.white)
        black_pen = QPen(Qt.black)
        green_pen = QPen(Qt.green)
        red_pen = QPen(Qt.red)
        blue_pen = QPen(Qt.blue)

        white_pen.setWidth(1)
        black_pen.setWidth(2)
        green_pen.setWidth(2)
        red_pen.setWidth(2)
        blue_pen.setWidth(2)

        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(self.width() / 2, self.height() / 2)

        # draw background

        painter.setPen(black_pen)
        painter.setBrush(bg_ground)

        painter.drawEllipse(-self._size / 2, -self._size / 2, self._size, self._size)

        # draw yaw lines
        painter.rotate(-self._yaw)  # could possibly move this before drawing NS arrow

        nyaw_lines = 36
        rot_ang = 360.0 / nyaw_lines
        yaw_line_leng = self._size / 25
        font_size = 12

        black_pen.setWidth(1)
        painter.setPen(black_pen)

        for i in range(0, nyaw_lines):
            if i == 0:
                s = "N"
                painter.setPen(blue_pen)
                painter.setFont(QFont("", font_size * 1.3))

            elif i == 9:
                s = "W"
                painter.setPen(black_pen)
                painter.setFont(QFont("", font_size * 1.3))

            elif i == 18:
                s = "S"
                painter.setPen(red_pen)
                painter.setFont(QFont("", font_size * 1.3))
            elif i == 27:
                s = "E"
                painter.setPen(black_pen)
                painter.setFont(QFont("", font_size * 1.3))
            else:
                s = str(i * rot_ang)
                painter.setPen(black_pen)
                painter.setFont(QFont("", font_size))

            fx1 = 0
            fy1 = -self._size / 2 + self._offset
            fx2 = 0

            if i % 3 == 0:
                fy2 = fy1 + yaw_line_leng
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2))

                fy2 = fy1 + yaw_line_leng + 4
                painter.drawText(QRectF(-50, fy2, 100, font_size + 2), Qt.AlignCenter, s)
            else:
                fy2 = fy1 + yaw_line_leng / 2
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2))

            painter.rotate(-rot_ang)
        painter.rotate(self._yaw)

        #  draw S/N arrow

        arrow_width = self._size / 5

        fx1 = 0
        fy1 = -self._size / 2 + self._offset + self._size / 25 + 15
        fx2 = -arrow_width / 2
        fy2 = 0
        fx3 = arrow_width / 2
        fy3 = 0

        painter.setPen(Qt.NoPen)

        painter.setBrush(QBrush(Qt.blue))

        points_n = [QPointF(fx1, fy1), QPointF(fx2, fy2), QPointF(fx3, fy3)]
        poly_n = QPolygon()
        for point in points_n:
            poly_n.append(point.toPoint())
        painter.drawPolygon(poly_n)

        fx1 = 0
        fy1 = self._size / 2 - self._offset - self._size / 25 - 15
        fx2 = -arrow_width / 2
        fy2 = 0
        fx3 = arrow_width / 2
        fy3 = 0

        painter.setBrush(QBrush(Qt.red))
        points_s = [QPointF(fx1, fy1), QPointF(fx2, fy2), QPointF(fx3, fy3)]
        poly_s = QPolygon()
        for point in points_s:
            poly_s.append(point.toPoint())
        painter.drawPolygon(poly_s)

        # draw yaw marker

        yaw_marker_size = self._size / 12

        painter.rotate(-self._yaw)  # could possibly move this before drawing NS arrow
        painter.setBrush(QBrush(QColor(0xFF, 0x00, 0x00, 0xE0)))

        fx1 = 0
        fy1 = -self._size / 2 + self._offset
        fx2 = fx1 - yaw_marker_size / 2
        fy2 = fy1 + yaw_marker_size
        fx3 = fx1 + yaw_marker_size / 2
        fy3 = fy1 + yaw_marker_size

        points_yaw = [QPointF(fx1, fy1), QPointF(fx2, fy2), QPointF(fx3, fy3)]
        poly_yaw = QPolygon()
        for point in points_yaw:
            poly_yaw.append(point.toPoint())
        painter.drawPolygon(poly_yaw)

        painter.rotate(self._yaw)

        alt_font_size = 13
        w = 130
        h = 2 * (alt_font_size + 8)
        fx = -w / 2
        fy = -h / 2

        black_pen.setWidth(2)
        painter.setPen(black_pen)
        painter.setBrush(QBrush(Qt.white))
        painter.setFont(QFont("", alt_font_size))

        painter.drawRoundedRect(fx, fy, w, h, 6, 6)
        painter.setPen(blue_pen)
        text = "ALT: " + str(self._alt) + "m"
        painter.drawText(QRectF(fx, fy + 2, w, h / 2), Qt.AlignCenter, text)

        text = "H: " + str(self._h) + "m"
        painter.drawText(QRectF(fx, fy + h / 2, w, h / 2), Qt.AlignCenter, text)

    def resizeEvent(self, QResizeEvent):
        """!@brief Resize Event

        Constrain the size to a square form
        @param self Python object pointer
        @param QResizeEvent Qt event
        """
        self._size = min(self.width(), self.height()) - 2 * self._offset


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = QCompass()
    win.setData(0, 0, 0)
    win.show()
    exit(app.exec_())
