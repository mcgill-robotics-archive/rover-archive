from PyQt5.QtCore import QObject

from controller.drive_controller import DriveController
from controller.joystick.joystick_controller import JoystickController


class MainController(QObject):
    def __init__(self, main_view, parent=None):
        super(MainController, self).__init__(parent)
        joystick_widget = main_view.joystick_mode_widget

        self.joystick_master = JoystickController(joystick_widget, self)
        self.drive_controller = DriveController(self)
        self.joystick_master.addController("Drive", self.drive_controller)
