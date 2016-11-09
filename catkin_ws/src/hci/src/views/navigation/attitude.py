"""!@brief Virtual horizon widget"""
import sys
from math import sqrt, atan

from PyQt5.QtCore import QPointF
from PyQt5.QtCore import QRectF
from PyQt5.QtCore import Qt
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtGui import QBrush
from PyQt5.QtGui import QColor
from PyQt5.QtGui import QFont
from PyQt5.QtGui import QPainter
from PyQt5.QtGui import QPen
from PyQt5.QtGui import QPolygon
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QWidget


class QAttitude(QWidget):
    """!@brief Virtual horizon widget that displays pitch and roll of robot"""
    canvasReplot = pyqtSignal(name="canvasReplot")

    def __init__(self, parent=None):
        """!@brief Constructor initializes member data and sets object properties

        @param self Python object pointer
        @param parent QWidget parent in Qt hierarchy
        """

        super(QAttitude, self).__init__(parent)

        self._size_min = 200
        self._size_max = 600
        self._offset = 2
        self._size = self._size_min - 2 * self._offset
        self._roll = 0.0
        self._pitch = 0.0

        self.setMinimumSize(self._size_min, self._size_min)
        self.setMaximumSize(self._size_max, self._size_max)
        self.resize(self._size_min, self._size_min)
        self.setFocusPolicy(Qt.NoFocus)

        self.canvasReplot.connect(self.canvasReplot_slot)

    @pyqtSlot(int, int)
    def setData(self, p, r):
        """!@brief Sets all member data

        @param self Python object pointer
        @param p The new pitch
        @param r the new roll
        """
        self.setPitch(p)
        self.setRoll(r)

    @pyqtSlot(int)
    def setPitch(self, val):
        """!@brief Set a new value for pitch.

        This is a qt slot so it is thread safe when called using the
        signal-slot mechanism.

        @param self Python object pointer
        @param val The new pitch value
        """
        self._pitch = val
        self._pitch = max(self._pitch, -90)
        self._pitch = min(self._pitch, 90)
        self.canvasReplot_slot()

    @pyqtSlot(int)
    def setRoll(self, val):
        """!@brief Set a new value for roll.

        This is a qt slot so it is thread safe when called using the
        signal-slot mechanism.

        @param self Python object pointer
        @param val The new roll value
        """
        self._roll = val
        self._roll = max(self._roll, -180)
        self._roll = min(self._roll, 180)
        self.canvasReplot_slot()

    def getPitch(self):
        return self._pitch

    def getRoll(self):
        return self._roll

    @pyqtSlot()
    def canvasReplot_slot(self):
        """!@brief Force a redraw

        @param self Python object pointer
        """
        self.update()

    def paintEvent(self, QPaintEvent):
        """!@Draw the horizon widget

        Paint event called by the Qt event handler

        @param self Python object pointer
        @param QPaintEvent Qt event
        """
        painter = QPainter(self)
        bg_sky = QBrush(QColor(48, 172, 220))
        bg_ground = QBrush(QColor(247, 168, 21))

        white_pen = QPen(Qt.white)
        black_pen = QPen(Qt.black)
        pitch_pen = QPen(Qt.white)
        pitch_zero = QPen(Qt.green)

        white_pen.setWidth(2)
        black_pen.setWidth(2)
        pitch_zero.setWidth(3)

        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(self.width() / 2, self.height() / 2)
        painter.rotate(self._roll)

        # FIXME: AHRS output left-hand values
        pitch_tem = -self._pitch

        # draw background
        y_min = self._size / 2 * -40.0 / 45.0
        y_max = self._size / 2 * 40.0 / 45.0

        y = self._size / 2 * pitch_tem / 45.0
        y = min(y, y_min)
        y = max(y, y_max)

        x = sqrt(self._size * self._size / 4 - y * y)
        gr = atan(y / x)

        painter.setPen(black_pen)
        painter.setBrush(bg_sky)
        painter.drawChord(-self._size / 2, -self._size / 2, self._size, self._size, gr * 16, (180 - 2 * gr) * 16)
        painter.setBrush(bg_ground)
        painter.drawChord(-self._size / 2, -self._size / 2, self._size, self._size, gr * 16, -(180 + 2 * gr) * 16)

        # draw pitch lines & marker
        ll = self._size / 8
        font_size = 12

        pitch_pen.setWidth(2)
        painter.setFont(QFont("", font_size))

        for i in range(-9, 9, 1):
            p = i * 10

            s = str(-p)
            if i % 3 == 0:
                l = ll
            else:
                l = ll / 2

            if i == 0:
                painter.setPen(pitch_zero)
                l *= 1.8
            else:
                painter.setPen(pitch_pen)

            y = self._size / 2 * p / 45.0 - self._size / 2 * pitch_tem / 45.0
            x = l

            r = sqrt(x * x + y * y)
            if r > self._size / 2:
                continue

            painter.drawLine(QPointF(-l, 1.0 * y), QPointF(l, 1.0 * y))

            text_width = 100

            if i % 3 == 0 and i != 0:
                painter.setPen(white_pen)

                x1 = -x - 2 - text_width
                y1 = y - font_size / 2 - 1
                painter.drawText(QRectF(x1, y1, text_width, font_size + 2), Qt.AlignRight | Qt.AlignVCenter, s)

        # draw markers

        marker_size = self._size / 20
        painter.setBrush(Qt.red)
        painter.setPen(Qt.NoPen)

        fx1 = marker_size
        fy1 = 0
        fx2 = fx1 + marker_size
        fy2 = -marker_size / 2
        fx3 = fx1 + marker_size
        fy3 = marker_size / 2

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
        n_roll_lines = 36
        rot_ang = 360.0 / n_roll_lines
        roll_line_length = self._size / 25
        black_pen.setWidth(1)
        painter.setPen(black_pen)
        painter.setFont(QFont("", font_size))

        for i in range(0, n_roll_lines):
            if i < n_roll_lines / 2:
                s = str(-i * rot_ang)
            else:
                s = str(360 - i * rot_ang)

            fx1 = 0
            fy1 = -self._size / 2 + self._offset
            fx2 = 0

            if i % 3 == 0:
                fy2 = fy1 + roll_line_length
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2))

                fy2 = fy1 + roll_line_length + 2
                painter.drawText(QRectF(-50, fy2, 100, font_size + 2), Qt.AlignCenter, s)

            else:
                fy2 = fy1 + roll_line_length / 2
                painter.drawLine(QPointF(fx1, fy1), QPointF(fx2, fy2))

            painter.rotate(rot_ang)

        # draw roll marker
        roll_marker_size = self._size / 25
        painter.rotate(-self._roll)
        painter.setBrush(QBrush(Qt.black))

        fx1 = 0
        fy1 = -self._size / 2 + self._offset
        fx2 = fx1 - roll_marker_size / 2
        fy2 = fy1 + roll_marker_size
        fx3 = fx1 + roll_marker_size / 2
        fy3 = fy1 + roll_marker_size

        points3 = [QPointF(fx1, fy1), QPointF(fx2, fy2), QPointF(fx3, fy3)]
        poly = QPolygon()
        for po in points3:
            poly.append(po.toPoint())
        painter.drawPolygon(poly)

    def resizeEvent(self, QResizeEvent):
        """!@brief Resize Event

        Constrain the size to a square form
        @param self Python object pointer
        @param QResizeEvent Qt event
        """
        self._size = min(self.width(), self.height()) - 2 * self._offset


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = QAttitude()
    win.setData(0, 0)
    win.show()
    exit(app.exec_())
