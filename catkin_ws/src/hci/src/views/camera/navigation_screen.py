"""!@brief Group several screens for the navigation"""

import sys

from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from views.camera.single_video_screen import SingleVideoScreen


class NavigationScreen(QWidget):
    """!@brief Multiple camera widget layered out for navigation tasks"""

    def __init__(self, parent=None):
        """!@brief Constructor

        @param self Python object pointer
        @param parent The Qt parent object
        """
        super(NavigationScreen, self).__init__(parent)

        ## Left vertical image screen
        self.left_wheel = SingleVideoScreen(270, self)
        ## Right vertical image screen
        self.right_wheel = SingleVideoScreen(90, self)
        ## Bottom image screen
        self.bottom_cam = SingleVideoScreen(0, self)

        horizontal = QHBoxLayout()
        horizontal.addWidget(self.left_wheel)
        horizontal.addWidget(self.right_wheel)
        vertical = QVBoxLayout()
        vertical.addItem(horizontal)
        vertical.addWidget(self.bottom_cam)
        self.setLayout(vertical)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = NavigationScreen()
    ui.show()
    exit(app.exec_())
