from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QWidget


class JoystickMode(QWidget):
    def __init__(self, parent=None):
        super(JoystickMode, self).__init__(parent)

        self.drive_button = QPushButton(self)
        self.arm_button = QPushButton(self)
        self.camera_button = QPushButton(self)
        self.drill_button = QPushButton(self)
