from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QWidget

from views.drive.drive_view import DriveView
from views.drive.wheel_status import WheelStatus
from views.joystick.joystick_mode import JoystickMode


class MainView(QWidget):
    def __init__(self, parent=None):
        super(MainView, self).__init__(parent)
        self.layout = QHBoxLayout()

        self.joystick_mode_widget = JoystickMode(self)
        self.drive_view = DriveView(self)
        self.layout.addWidget(self.joystick_mode_widget)
        self.layout.addWidget(self.drive_view)

        self.setLayout(self.layout)