"""!@brief Main navigation view widget.

Should be the only module needed in the top level view.
"""
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QWidget

from views.navigation.attitude import QAttitude
from views.navigation.compass import QCompass


class NavigationView(QWidget):
    def __init__(self, parent=QWidget):
        super(NavigationView, self).__init__(parent)
        hbox = QHBoxLayout()

        self._alt = QAttitude(self)
        self._comp = QCompass(self)

        hbox.addWidget(self._alt)
        hbox.addWidget(self._comp)
        self.setLayout(hbox)

    def handle_new_pitch(self, val):
        self._alt.setPitch(val)

    def handle_new_roll(self, val):
        self._alt.setRoll(val)

    def handle_new_yaw(self, val):
        self._comp.setYaw(val)
