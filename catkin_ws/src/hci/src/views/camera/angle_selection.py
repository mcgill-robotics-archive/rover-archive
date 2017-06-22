"""!@brief Simple selector widget with specific angle options"""
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QRadioButton
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QWidget


class AngleSelection(QWidget):
    """!@brief Widget offers a series of angles to chose from.

    Signal is emitted with the selected angle when the selection changes.
    """
    ## Dictates the desired angle
    turnAngle = pyqtSignal(int, name="turnAngle")

    def __init__(self, angle=0, parent=None):
        """!@brief Constructor lays out and initialises data

        @param self Python object pointer
        @param angle Starting angle for the selector
        @param parent Qt parent object
        """
        super(AngleSelection, self).__init__(parent)

        layout = QHBoxLayout()
        self._deg0 = QRadioButton(self)
        self._deg90 = QRadioButton(self)
        self._deg180 = QRadioButton(self)
        self._deg270 = QRadioButton(self)

        layout.addWidget(self._deg0)
        layout.addWidget(self._deg90)
        layout.addWidget(self._deg180)
        layout.addWidget(self._deg270)

        self._deg0.setText("0" + unichr(176))
        self._deg90.setText("90" + unichr(176))
        self._deg180.setText("180" + unichr(176))
        self._deg270.setText("270" + unichr(176))

        self.setLayout(layout)
        self._deg0.toggled.connect(self._emit_signal)
        self._deg90.toggled.connect(self._emit_signal)
        self._deg180.toggled.connect(self._emit_signal)
        self._deg270.toggled.connect(self._emit_signal)

        self._set_default_angle(angle)

    def _set_default_angle(self, angle):
        """!@brief Initialise the widget and click the proper button.

        @param self Python object pointer
        @param angle Current angle for the selector
        """
        if angle == 90:
            self._deg90.click()
        elif angle == 180:
            self._deg180.click()
        elif angle == 270:
            self._deg270.click()
        else:
            self._deg0.click()

    def _emit_signal(self):
        """!@brief Handle a new angle. Emit the proper signal

        @param self Python object pointer
        """
        if self._deg90.isChecked():
            angle = 90

        elif self._deg180.isChecked():
            angle = 180

        elif self._deg270.isChecked():
            angle = 270

        else:
            angle = 0

        self.turnAngle.emit(angle)
