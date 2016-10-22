"""run in dedicated thread"""
from PyQt5.QtCore import QThread


class JoystickAcquisition(QThread):
    def __init__(self, parent=None):
        super(JoystickAcquisition, self).__init__(parent)
