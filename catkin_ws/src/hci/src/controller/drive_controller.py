from PyQt5.QtCore import pyqtSlot

from controller.joystick.joystick_base import JoystickBase
from controller.joystick.joystick_data import JoystickData


class DriveController(JoystickBase):
    @pyqtSlot(JoystickData)
    def handle_joystick_data(self, data):
        print("x: {0}, y: {1}".format(data.a1, data.a2))
        pass

    def __init__(self, parent=None):
        super(DriveController, self).__init__(parent)
        pass
