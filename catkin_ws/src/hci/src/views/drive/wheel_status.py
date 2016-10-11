

## Wheel Status Structure
#
# Group the status of the wheels to make update easier
import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QFrame
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from utilities import lbl_bg_grn, lbl_bg_red


class WheelStatusStruct(object):
    def __init__(self, fl=False, ml=False, bl=False, fr=False, mr=False, br=False):
        self.fl = fl
        self.ml = ml
        self.bl = bl
        self.fr = fr
        self.mr = mr
        self.br = br


## Helper function to automate the changing of background color based on boolean
#
# @param item qt object which will be changed
# @param value boolean value, true to make status green, false to red
def set_color_label(item, value):
    if value:
        lbl_bg_grn(item, "OK")
    else:
        lbl_bg_red(item, "BAD")


## Indicate the status of the wheels
#
class WheelStatus(QWidget):
    def __init__(self, parent=None):
        super(WheelStatus, self).__init__(parent)

        vbox1 = QVBoxLayout()
        vbox1.setContentsMargins(0, 0, 0, 0)

        grid1 = QGridLayout()
        grid1.setContentsMargins(0, 0, 0, 0)

        label = QLabel(self)
        label.setText("Wheel Status")

        self.fl_ok = QLabel(self)
        self.fl_ok.setAlignment(Qt.AlignCenter)
        grid1.addWidget(self.fl_ok, 0, 0, 1, 1)

        self.mr_ok = QLabel(self)
        self.mr_ok.setAlignment(Qt.AlignCenter)
        grid1.addWidget(self.mr_ok, 1, 1, 1, 1)

        self.fr_ok = QLabel(self)
        self.fr_ok.setAlignment(Qt.AlignCenter)
        grid1.addWidget(self.fr_ok, 0, 1, 1, 1)

        self.ml_ok = QLabel(self)
        self.ml_ok.setAlignment(Qt.AlignCenter)
        grid1.addWidget(self.ml_ok, 1, 0, 1, 1)

        self.bl_ok = QLabel(self)
        self.bl_ok.setAlignment(Qt.AlignCenter)
        grid1.addWidget(self.bl_ok, 2, 0, 1, 1)

        self.br_ok = QLabel(self)
        self.br_ok.setAlignment(Qt.AlignCenter)
        grid1.addWidget(self.br_ok, 2, 1, 1, 1)

        line_2 = QFrame(self)
        line_2.setFrameShape(QFrame.HLine)
        line_2.setFrameShadow(QFrame.Sunken)

        vbox1.addWidget(label)
        vbox1.addWidget(line_2)
        vbox1.addItem(grid1)
        self.setLayout(vbox1)

        self.update_motor_status(WheelStatusStruct())

    ## Use status structure <code>WheelStatusStruct</code> to update each wheel background
    #
    # @param status <code>WheelStatusStruct</code> with the status of each wheel
    def update_motor_status(self, status):
        set_color_label(self.fl_ok, status.fl)
        set_color_label(self.ml_ok, status.ml)
        set_color_label(self.bl_ok, status.bl)
        set_color_label(self.fr_ok, status.fr)
        set_color_label(self.mr_ok, status.mr)
        set_color_label(self.br_ok, status.br)


if __name__ == "__main__":
    from PyQt5.QtCore import pyqtSignal

    class WheelDisplayTest(QWidget):
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
