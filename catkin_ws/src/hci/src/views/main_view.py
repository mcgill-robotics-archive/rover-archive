from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from views.arm.arm_view import ArmView
from views.camera.navigation_screen import NavigationScreen
from views.drive.drive_view import DriveView
from views.navigation.map import Map
from views.navigation.pose_display import PoseDisplay
from views.joystick.joystick_mode import JoystickMode
from views.navigation.navigation_view import NavigationView
from views.autonomous.autonomous_mode_selection import AutonomousModeSelection


class MainView(QWidget):
    """The main view module."""

    def __init__(self, parent=None):
        super(MainView, self).__init__(parent)
        h1 = QHBoxLayout()
        h2 = QHBoxLayout()
        v1 = QVBoxLayout()

        self.nav_mode = AutonomousModeSelection(self)
        self.joystick_mode_widget = JoystickMode(self)
        self.drive_view = DriveView(self)
        self.navigation_view = NavigationView(self)
        self.pose_display = PoseDisplay(self)
        self.nav_screen = NavigationScreen(self)
        # self.map = Map(self)
        self.arm_view = ArmView(self)


        h1.addWidget(self.joystick_mode_widget)
        h1.addWidget(self.drive_view)
        v1.addWidget(self.nav_mode)
        v1.addItem(h1)
        v1.addWidget(self.arm_view)
        v1.addWidget(self.navigation_view)
        v1.addWidget(self.pose_display)

        h2.addWidget(self.nav_screen)
        # h2.addWidget(self.map)
        h2.addItem(v1)
        self.setLayout(h2)
