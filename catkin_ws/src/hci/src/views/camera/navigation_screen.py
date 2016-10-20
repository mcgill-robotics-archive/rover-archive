import sys

from PyQt5.QtGui import QImage
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from views.camera.single_video_screen import SingleVideoScreen


class NavigationScreen(QWidget):
    def __init__(self, parent=None):
        super(NavigationScreen, self).__init__(parent)
        self.left_wheel = SingleVideoScreen(270, self)
        self.right_wheel = SingleVideoScreen(90, self)
        self.bottom_cam = SingleVideoScreen(0, self)
        self.left_wheel.setControlsVisible(False)
        self.right_wheel.setControlsVisible(False)

        horizontal = QHBoxLayout()
        horizontal.addWidget(self.left_wheel)
        horizontal.addWidget(self.right_wheel)
        vertical = QVBoxLayout()
        vertical.addItem(horizontal)
        vertical.addWidget(self.bottom_cam)
        self.setLayout(vertical)

        image = QImage("/home/david/rover/3_Main_Inverted.png")
        self.left_wheel.newSample(image)
        self.right_wheel.newSample(image)
        self.bottom_cam.newSample(image)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = NavigationScreen()
    ui.show()
    exit(app.exec_())
