import sys

from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QFrame
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget


if sys.version_info >= (3,0):
    deg_symb = chr(176)
else:
    deg_symb = unichr(176)


class DCDC_Information(QWidget):
    def __init__(self, parent=None):
        super(DCDC_Information, self).__init__(parent)

        oc_title = QLabel("Output Current:", self)
        op_title = QLabel("Output Power:", self)
        ov_title = QLabel("Output Voltage:", self)
        ic_title = QLabel("Input Current:", self)
        iv_title = QLabel("Input Voltage:", self)
        temp_title = QLabel("Temparature:", self)
        self.oc_lbl = QLabel(self)
        self.ov_lbl = QLabel(self)
        self.op_lbl = QLabel(self)
        self.ic_lbl = QLabel(self)
        self.iv_lbl = QLabel(self)
        self.temp_lbl = QLabel(self)

        grid = QGridLayout()
        grid.addWidget(iv_title, 0, 0)
        grid.addWidget(ic_title, 1, 0)
        grid.addWidget(temp_title, 2, 0)
        grid.addWidget(ov_title, 0, 2)
        grid.addWidget(oc_title, 1, 2)
        grid.addWidget(op_title, 2, 2)

        grid.addWidget(self.iv_lbl, 0, 1)
        grid.addWidget(self.ic_lbl, 1, 1)
        grid.addWidget(self.temp_lbl, 2, 1)
        grid.addWidget(self.ov_lbl, 0, 3)
        grid.addWidget(self.oc_lbl, 1, 3)
        grid.addWidget(self.op_lbl, 2, 3)

        wind_title = QLabel("Power Supply Information:", self)
        title_font = QFont()
        title_font.setPointSize(15)
        title_font.setBold(True)
        title_font.setWeight(75)
        wind_title.setFont(title_font)

        line_1 = QFrame(self)
        line_1.setFrameShape(QFrame.HLine)
        line_1.setFrameShadow(QFrame.Sunken)

        v1 = QVBoxLayout()
        v1.addWidget(line_1)
        v1.addWidget(wind_title)
        v1.addLayout(grid)

        self.setLayout(v1)

    @pyqtSlot(float)
    def update_iv(self, value):
        self.iv_lbl.setText(str(value) + "V")

    @pyqtSlot(float)
    def update_ov(self, value):
        self.ov_lbl.setText(str(value) + "V")

    @pyqtSlot(float)
    def update_ic(self, value):
        self.ic_lbl.setText(str(value) + "A")

    @pyqtSlot(float)
    def update_oc(self, value):
        self.oc_lbl.setText(str(value) + "A")

    @pyqtSlot(float)
    def update_op(self, value):
        self.op_lbl.setText(str(value) + "W")

    @pyqtSlot(float)
    def update_temp(self, value):
        self.temp_lbl.setText(str(value) + deg_symb)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = DCDC_Information()
    ui.show()
    ui.update_iv(10)
    sys.exit(app.exec_())
