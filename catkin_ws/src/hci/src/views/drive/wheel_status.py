"""!@brief Wheel status module is meant to display the current readiness
of the wheel and specifically motor controllers

The module includes the display class and the data structure used to
update the display
"""

import sys
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QFrame
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from utilities import lbl_bg_grn, lbl_bg_red


class WheelStatusStruct(object):
    """!@brief Group the status of the wheels to make update easier
    """
    def __init__(self, fl=False, ml=False, bl=False, fr=False, mr=False, br=False):
        """!@brief Constructor

        @param self Python object pointer
        @param fl front left status
        @param ml middle left status
        @param bl back left status
        @param fr front right status
        @param mr middle right status
        @param br back right status
        """

        self.fl = fl
        self.ml = ml
        self.bl = bl
        self.fr = fr
        self.mr = mr
        self.br = br


def set_color_label(item, value):
    """!@brief Helper function to automate the changing of background color
    based on boolean

    @param item qt object which will be changed
    @param value boolean value, true to make status green, false to red
    """
    if value:
        lbl_bg_grn(item, "OK")
    else:
        lbl_bg_red(item, "BAD")


class WheelStatus(QWidget):
    """!@brief Indicate the status of the wheels
    """
    def __init__(self, parent=None):
        """!@brief Constructor, creates labels and sets layout
        @param self Python object pointer
        @param parent Qt parent
        """
        super(WheelStatus, self).__init__(parent)

        vbox1 = QVBoxLayout()
        vbox1.setContentsMargins(0, 0, 0, 0)

        grid1 = QGridLayout()
        grid1.setContentsMargins(0, 0, 0, 0)

        label = QLabel(self)
        label.setText("Wheel Status")

        self._fl_ok = QLabel(self)
        self._fl_ok.setAlignment(Qt.AlignCenter)
        grid1.addWidget(self._fl_ok, 0, 0, 1, 1)

        self._mr_ok = QLabel(self)
        self._mr_ok.setAlignment(Qt.AlignCenter)
        grid1.addWidget(self._mr_ok, 1, 1, 1, 1)

        self._fr_ok = QLabel(self)
        self._fr_ok.setAlignment(Qt.AlignCenter)
        grid1.addWidget(self._fr_ok, 0, 1, 1, 1)

        self._ml_ok = QLabel(self)
        self._ml_ok.setAlignment(Qt.AlignCenter)
        grid1.addWidget(self._ml_ok, 1, 0, 1, 1)

        self._bl_ok = QLabel(self)
        self._bl_ok.setAlignment(Qt.AlignCenter)
        grid1.addWidget(self._bl_ok, 2, 0, 1, 1)

        self._br_ok = QLabel(self)
        self._br_ok.setAlignment(Qt.AlignCenter)
        grid1.addWidget(self._br_ok, 2, 1, 1, 1)

        line_2 = QFrame(self)
        line_2.setFrameShape(QFrame.HLine)
        line_2.setFrameShadow(QFrame.Sunken)

        vbox1.addWidget(label)
        vbox1.addWidget(line_2)
        vbox1.addItem(grid1)
        self.setLayout(vbox1)

        self.update_motor_status(WheelStatusStruct())

    @pyqtSlot(WheelStatusStruct)
    def update_motor_status(self, status):
        """!@brief Use status structure <code>WheelStatusStruct</code> to update each wheel background

        @param status <code>WheelStatusStruct</code> with the status of each wheel
        @param self Python object pointer
        """
        set_color_label(self._fl_ok, status.fl)
        set_color_label(self._ml_ok, status.ml)
        set_color_label(self._bl_ok, status.bl)
        set_color_label(self._fr_ok, status.fr)
        set_color_label(self._mr_ok, status.mr)
        set_color_label(self._br_ok, status.br)


if __name__ == "__main__":
    from PyQt5.QtCore import pyqtSignal

    class WheelDisplayTest(QWidget):
        """!@brief Test class that displays the WheelStatus window
        """
        signal = pyqtSignal(WheelStatusStruct, name="test_signal")

        def __init__(self):
            QWidget.__init__(self)
            self.ui = WheelStatus(self)
            self.signal.connect(self.ui.update_motor_status)


    app = QApplication(sys.argv)
    ui = WheelDisplayTest()
    ui.show()
    ui.signal.emit(WheelStatusStruct(False, False, True))

    sys.exit(app.exec_())
