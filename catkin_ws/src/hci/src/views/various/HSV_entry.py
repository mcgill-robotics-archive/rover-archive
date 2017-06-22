from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QSpinBox
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

class HSVParams(object):
    h_low = 0
    h_high = 0
    s_low = 0
    s_high = 0
    v_low = 0
    v_high = 0


class HSVEntry(QWidget):
    updateParams = pyqtSignal(HSVParams)
    def __init__(self, parent=None):
        super(HSVEntry, self).__init__(parent)

        title_lbl = QLabel("Camera settings", self)
        title_font = QFont()
        title_font.setPointSize(15)
        title_font.setBold(True)
        title_font.setWeight(75)
        title_lbl.setFont(title_font)

        grid = QGridLayout()
        grid.addWidget(QLabel("Hue Low", self), 0, 0)
        grid.addWidget(QLabel("Hue High", self), 1, 0)
        grid.addWidget(QLabel("Saturation Low", self), 2, 0)
        grid.addWidget(QLabel("Saturation High", self), 3, 0)
        grid.addWidget(QLabel("Value Low", self), 0, 2)
        grid.addWidget(QLabel("Value High", self), 1, 2)

        self.h_low_lbl = QSpinBox(self)
        grid.addWidget(self.h_low_lbl, 0, 1)

        self.h_high_lbl = QSpinBox(self)
        grid.addWidget(self.h_high_lbl, 1, 1)

        self.s_low_lbl = QSpinBox(self)
        grid.addWidget(self.s_low_lbl, 2, 1)

        self.s_high_lbl = QSpinBox(self)
        grid.addWidget(self.s_high_lbl, 3, 1)

        self.v_low_lbl = QSpinBox(self)
        grid.addWidget(self.v_low_lbl, 0, 3)

        self.v_high_lbl = QSpinBox(self)
        grid.addWidget(self.v_high_lbl, 1, 3)

        v = QVBoxLayout()
        v.addWidget(title_lbl)
        v.addItem(grid)

        self.h_low_lbl.valueChanged.connect(self.onClick)
        self.h_high_lbl.valueChanged.connect(self.onClick)
        self.v_low_lbl.valueChanged.connect(self.onClick)
        self.v_high_lbl.valueChanged.connect(self.onClick)
        self.s_low_lbl.valueChanged.connect(self.onClick)
        self.s_high_lbl.valueChanged.connect(self.onClick)

        self.setLayout(v)

    def onClick(self):
        params = HSVParams()
        params.h_high = self.h_high_lbl.value()
        params.h_low = self.h_low_lbl.value()
        params.s_high = self.s_high_lbl.value()
        params.s_low = self.s_low_lbl.value()
        params.v_high = self.v_high_lbl.value()
        params.v_low = self.v_low_lbl.value()

        self.updateParams.emit(params)
