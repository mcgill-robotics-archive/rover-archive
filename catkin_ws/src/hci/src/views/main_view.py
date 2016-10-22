from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QWidget

from views.joystick.joystick_mode import JoystickMode


class MainView(QWidget):
    def __init__(self, parent=None):
        super(MainView, self).__init__(parent)
        self.layout = QHBoxLayout()

        self.joystick_mode_widget = JoystickMode(self)
        self.layout.addWidget(self.joystick_mode_widget)