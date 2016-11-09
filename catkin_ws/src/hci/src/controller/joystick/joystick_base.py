"""!@brief Simply class interface setting minimum requirements for controller
classes"""

from PyQt5.QtCore import QObject


class JoystickBase(QObject):
    """!@brief Base class for controller that need to acquire joystick inputs
    at some point or another.

    Class implementing this interface are available for joystick data
    forwarding via the abstract slot in this interface.
    """

    def handle_joystick_data(self, data):
        """!@brief Interface function connected to the joystick acquisition
        signal by joystick controller.

        @param data JoystickData object with latest joystick data status.
        """
        raise NotImplementedError("Function not implemented")
