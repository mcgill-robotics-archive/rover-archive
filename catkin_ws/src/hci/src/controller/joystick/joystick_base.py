from PyQt5.QtCore import QObject


class JoystickBase(QObject):
    def handle_joystick_data(self, data):
        raise NotImplementedError("Function not implemented")
