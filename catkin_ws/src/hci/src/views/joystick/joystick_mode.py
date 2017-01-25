"""!@brief Mode selection widget

"""

import sys
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QWidget


class JoystickMode(QWidget):
    """!@brief Mode selection widget for joystick mode selection

    Signal should be handled by the JoystickController to enable certain
    JoystickBase to acquire inputs

    """

    ## Signal emitted when someone select a new mode is selected
    changeMode = pyqtSignal(str)

    def __init__(self, parent=None):
        """!@brief Constructor creates the layout and widget list

        Will wait to receive controllers to register

        @param self Python object pointer
        @param parent Qt object hierarchy
        """
        super(JoystickMode, self).__init__(parent)

        self._button_list = []
        self._layout = QHBoxLayout()
        self.setLayout(self._layout)

    @pyqtSlot(str)
    def addMode(self, name):
        """!@brief Slot to add joystick modes to the display.

        Every mode is a different button on screen.

        @param self Python object pointer
        @param name String with the name of the mode, will be the display text
        on the button
        """
        button = QPushButton(name, self)
        button.setCheckable(True)
        button.setAutoExclusive(True)
        self._layout.addWidget(button)

        def button_callback():
            """!@brief Callback function created for every new button.

            This function will emit the changeMode signal with the proper
            button name when clicked
            """
            name_text = button.text()
            self.changeMode.emit(name_text)

        button.clicked.connect(button_callback)


def callback(string):
    """!@brief Test callback"""
    print(string)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = JoystickMode()
    ui.changeMode.connect(callback)
    ui.show()
    ui.addMode("David")
    ui.addMode("David L-B")
    ui.addMode("David Lavoie")
    ui.addMode("David Boutin")
    exit(app.exec_())
