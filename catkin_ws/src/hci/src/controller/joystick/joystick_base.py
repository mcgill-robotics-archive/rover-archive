from PyQt5.QtCore import QObject, pyqtSlot

from controller.joystick.joystick_data import JoystickData


class JoystickBase(QObject):
    @pyqtSlot(JoystickData)
    def handle_joystick_data(self, data):
        raise NotImplementedError("Function not implemented")
