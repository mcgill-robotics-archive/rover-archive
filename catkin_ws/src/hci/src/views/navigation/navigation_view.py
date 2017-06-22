"""!@brief Main navigation view widget.

Should be the only module needed in the top level view.
"""
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QWidget

from views.navigation.attitude import QAttitude
from views.navigation.compass import QCompass


class NavigationView(QWidget):
    """!@brief Top level navigation widget, includes and lays out compass and
    attitude display widgets."""

    def __init__(self, parent=QWidget):
        """!@brief Constructor instantiates sub widgets

        @param self Python object pointer
        @param parent Qt object hierarchy parent
        """
        super(NavigationView, self).__init__(parent)
        hbox = QHBoxLayout()

        self._alt = QAttitude(self)
        self._comp = QCompass(self)

        hbox.addWidget(self._alt)
        hbox.addWidget(self._comp)
        self.setLayout(hbox)

    def handle_new_pitch(self, val):
        """!@brief Forward new pitch value for display

        @param self Python object pointer
        @param val The new pitch value
        """
        self._alt.setPitch(val)

    def handle_new_roll(self, val):
        """!@brief Forward new roll value for display

        @param self Python object pointer
        @param val The new roll value
        """
        self._alt.setRoll(val)

    def handle_new_yaw(self, val):
        """!@brief Forward new yaw value for display

        @param self Python object pointer
        @param val The new yaw value
        """
        self._comp.setYaw(val)
