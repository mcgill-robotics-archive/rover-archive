from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from views.drive.drive_view import DriveView
from views.map.pose_display import PoseDisplay
from views.drive.wheel_status import WheelStatus
from views.joystick.joystick_mode import JoystickMode
from views.navigation.navigation_view import NavigationView


class MainView(QWidget):
    """The main view module."""

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
        self.pose_display = PoseDisplay(self)
        self.layout.addWidget(self.joystick_mode_widget)
        self.layout.addWidget(self.drive_view)
        self.layout.addWidget(self.pose_display)

        self.setLayout(self.layout)
