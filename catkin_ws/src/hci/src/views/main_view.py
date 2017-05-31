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
from views.science.sampling_information import SamplingInformation
from views.various.autonomous_mode_selection import AutonomousModeSelection
from views.various.dcdc_view import DCDC_Information


class MainView(QWidget):
    """The main view module."""

    def __init__(self, map_enable=False, parent=None):
        super(MainView, self).__init__(parent)
        h1 = QHBoxLayout()
        h2 = QHBoxLayout()
        v1 = QVBoxLayout()
        v2 = QVBoxLayout()

        self.nav_mode = AutonomousModeSelection(self)
        self.joystick_mode_widget = JoystickMode(self)
        self.drive_view = DriveView(self)
        self.navigation_view = NavigationView(self)
        self.pose_display = PoseDisplay(self)
        self.nav_screen = NavigationScreen(self)
        self.arm_view = ArmView(self)
        self.power_info = DCDC_Information(self)
        self.science_info = SamplingInformation(self)

        v2.addWidget(self.nav_mode)
        v2.addWidget(self.joystick_mode_widget)
        h1.addItem(v2)
        h1.addWidget(self.drive_view)

        v1.addItem(h1)
        v1.addWidget(self.arm_view)
        v1.addWidget(self.science_info)
        v1.addWidget(self.navigation_view)
        v1.addWidget(self.pose_display)
        v1.addWidget(self.power_info)

        h2.addWidget(self.nav_screen)

        if map_enable:
            self.map = Map(self)
            h2.addWidget(self.map)
        h2.addItem(v1)
        self.setLayout(h2)
