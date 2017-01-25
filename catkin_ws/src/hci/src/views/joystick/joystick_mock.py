import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtCore import Qt
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QCheckBox
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QSlider
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from model.joystick_data import JoystickData


class JoystickMock(QWidget):
    joystickDataUpdated = pyqtSignal(object)

    def __init__(self, parent=None):
        super(QWidget, self).__init__()

        h1 = QHBoxLayout(self)
        v1 = QVBoxLayout()
        v2 = QVBoxLayout()
        v3 = QVBoxLayout()
        v4 = QVBoxLayout()

        self._a1_slider = QSlider(Qt.Vertical, self)
        self._a2_slider = QSlider(Qt.Vertical, self)
        self._a3_slider = QSlider(Qt.Vertical, self)
        self._a4_slider = QSlider(Qt.Vertical, self)
        self._a1_slider.setRange(-1000, 1000)
        self._a2_slider.setRange(-1000, 1000)
        self._a3_slider.setRange(-1000, 1000)
        self._a4_slider.setRange(-1000, 1000)

        v1.addWidget(self._a1_slider)
        v1.addWidget(QLabel("A1", self))
        v2.addWidget(self._a2_slider)
        v2.addWidget(QLabel("A2", self))
        v3.addWidget(self._a3_slider)
        v3.addWidget(QLabel("A3", self))
        v4.addWidget(self._a4_slider)
        v4.addWidget(QLabel("A4", self))

        g1 = QGridLayout()

        self._b1 = QPushButton("B1, Enable", self)
        self._b2 = QPushButton("B2, Ackerman", self)
        self._b3 = QPushButton("B3, Point", self)
        self._b4 = QPushButton("B4, Translate", self)
        self._b5 = QPushButton("B5", self)
        self._b6 = QPushButton("B6", self)
        self._b7 = QPushButton("B7, Open Loop", self)
        self._b8 = QPushButton("B8, Closed Loop", self)
        self._b9 = QPushButton("B9", self)
        self._b10 = QPushButton("B10", self)
        self._b11 = QPushButton("B11", self)
        self._b12 = QPushButton("B12", self)

        g1.addWidget(self._b1, 0, 0)
        g1.addWidget(self._b2, 0, 1)
        g1.addWidget(self._b3, 0, 2)
        g1.addWidget(self._b4, 0, 3)
        g1.addWidget(self._b5, 1, 0)
        g1.addWidget(self._b6, 1, 1)
        g1.addWidget(self._b7, 1, 2)
        g1.addWidget(self._b8, 1, 3)
        g1.addWidget(self._b9, 2, 0)
        g1.addWidget(self._b10, 2, 1)
        g1.addWidget(self._b11, 2, 2)
        g1.addWidget(self._b12, 2, 3)

        h1.addLayout(v1)
        h1.addLayout(v2)
        h1.addLayout(v3)
        h1.addLayout(v4)
        h1.addLayout(g1)

        self._b1.pressed.connect(self.update_status)
        self._b2.pressed.connect(self.update_status)
        self._b3.pressed.connect(self.update_status)
        self._b4.pressed.connect(self.update_status)
        self._b5.pressed.connect(self.update_status)
        self._b6.pressed.connect(self.update_status)
        self._b7.pressed.connect(self.update_status)
        self._b8.pressed.connect(self.update_status)
        self._b9.pressed.connect(self.update_status)
        self._b10.pressed.connect(self.update_status)
        self._b11.pressed.connect(self.update_status)
        self._b12.pressed.connect(self.update_status)

        self._b1.released.connect(self.update_status)
        self._b2.released.connect(self.update_status)
        self._b3.released.connect(self.update_status)
        self._b4.released.connect(self.update_status)
        self._b5.released.connect(self.update_status)
        self._b6.released.connect(self.update_status)
        self._b7.released.connect(self.update_status)
        self._b8.released.connect(self.update_status)
        self._b9.released.connect(self.update_status)
        self._b10.released.connect(self.update_status)
        self._b11.released.connect(self.update_status)
        self._b12.released.connect(self.update_status)

        self._a1_slider.valueChanged.connect(self.update_status)
        self._a2_slider.valueChanged.connect(self.update_status)
        self._a3_slider.valueChanged.connect(self.update_status)
        self._a4_slider.valueChanged.connect(self.update_status)

        self.return_value = QCheckBox("Slider Return to 0", self)
        g1.addWidget(self.return_value, 3, 0)

        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_status)
        self.timer.start()

        self._a1_slider.sliderReleased.connect(self.set_slider)
        self._a2_slider.sliderReleased.connect(self.set_slider)
        self._a3_slider.sliderReleased.connect(self.set_slider)
        self._a4_slider.sliderReleased.connect(self.set_slider)

    def set_slider(self):
        if self.return_value.isChecked():
            self._a1_slider.setValue(0)
            self._a2_slider.setValue(0)
            self._a3_slider.setValue(0)
            self._a4_slider.setValue(0)

    def update_status(self):
        data = JoystickData()

        data.a1 = self._a1_slider.value()/1000.0
        data.a2 = self._a2_slider.value()/1000.0
        data.a3 = self._a3_slider.value()/1000.0
        data.a4 = self._a4_slider.value()/1000.0

        data.b1 = self._b1.isDown()
        data.b2 = self._b2.isDown()
        data.b3 = self._b3.isDown()
        data.b4 = self._b4.isDown()
        data.b5 = self._b5.isDown()
        data.b6 = self._b6.isDown()
        data.b7 = self._b7.isDown()
        data.b8 = self._b8.isDown()
        data.b9 = self._b9.isDown()
        data.b10 = self._b10.isDown()
        data.b11 = self._b11.isDown()
        data.b12 = self._b12.isDown()

        self.joystickDataUpdated.emit(data)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ui = JoystickMock()
    ui.show()
    sys.exit(app.exec_())