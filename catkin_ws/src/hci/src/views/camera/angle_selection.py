from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QRadioButton
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QWidget


class AngleSelection(QWidget):
    turnAngle = pyqtSignal(int, name="turnAngle")

    def __init__(self, angle=0,parent=None):
        super(AngleSelection, self).__init__(parent)

        layout = QHBoxLayout()
        self.deg0 = QRadioButton(self)
        self.deg90 = QRadioButton(self)
        self.deg180 = QRadioButton(self)
        self.deg270 = QRadioButton(self)

        layout.addWidget(self.deg0)
        layout.addWidget(self.deg90)
        layout.addWidget(self.deg180)
        layout.addWidget(self.deg270)

        self.deg0.setText("0" + unichr(176))
        self.deg90.setText("90" + unichr(176))
        self.deg180.setText("180" + unichr(176))
        self.deg270.setText("270" + unichr(176))

        self.setLayout(layout)
        self.deg0.toggled.connect(self._emit_signal)
        self.deg90.toggled.connect(self._emit_signal)
        self.deg180.toggled.connect(self._emit_signal)
        self.deg270.toggled.connect(self._emit_signal)

        self._set_default_angle(angle)

    def _set_default_angle(self, angle):
        if angle == 90:
            self.deg90.click()
        elif angle == 180:
            self.deg180.click()
        elif angle == 270:
            self.deg270.click()
        else:
            self.deg0.click()

    def _emit_signal(self):
        if self.deg90.isChecked():
            angle = 90

        elif self.deg180.isChecked():
            angle = 180

        elif self.deg270.isChecked():
            angle = 270

        else:
            angle = 0

        self.turnAngle.emit(angle)

