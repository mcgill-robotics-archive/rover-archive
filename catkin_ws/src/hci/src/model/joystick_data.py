"""!@brief Dictionary-type data structure for joystick button states."""

from PyQt5.QtCore import QObject


class JoystickData(QObject):
    """!@brief Class to structure and pass around the button states

    Should be used as a structure, does not follow any good encapsulation
    practices, this is voluntary for simplicity and delegate interpretation
    and parsing to the handler classes.
    """
    b1 = False
    b2 = False
    b3 = False
    b4 = False
    b5 = False
    b6 = False
    b7 = False
    b8 = False
    b9 = False
    b10 = False
    b11 = False
    b12 = False

    hat = 0

    hat_left = False
    hat_top = False
    hat_down = False
    hat_right = False

    a1 = 0
    a2 = 0
    a3 = 0
    a4 = 0

    def __init__(self):
        QObject.__init__(self)
