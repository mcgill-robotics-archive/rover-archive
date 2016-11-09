from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from views.drive.drive_view import DriveView
from views.drive.wheel_status import WheelStatus
from views.joystick.joystick_mode import JoystickMode
from views.navigation.navigation_view import NavigationView


class MainView(QWidget):
    def __init__(self, parent=None):
        super(MainView, self).__init__(parent)
        h1 = QHBoxLayout()
        v1 = QVBoxLayout()


        self.joystick_mode_widget = JoystickMode(self)
        self.drive_view = DriveView(self)
        self.navigation_view = NavigationView(self)

        h1.addWidget(self.joystick_mode_widget)
        h1.addWidget(self.drive_view)
        v1.addItem(h1)
        v1.addWidget(self.navigation_view)

        self.setLayout(v1)
