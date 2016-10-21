import sys

from PyQt5.QtCore import Qt
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from PyQt5.QtGui import QTransform
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
from PyQt5.QtWidgets import QComboBox

from views.camera.angle_selection import AngleSelection


class SingleVideoScreen(QWidget):
    playTopic = pyqtSignal(str)

    def __init__(self, angle=0, parent=None):
        super(SingleVideoScreen, self).__init__(parent)

        self.imageDisplay = QLabel(self)
        self.imageDisplay.setMinimumSize(100, 100)
        self.topic_selector = QComboBox(self)
        self.angle_selector = AngleSelection(angle, self)
        self.angle = angle

        hbox = QVBoxLayout()
        self.setLayout(hbox)

        hbox.addWidget(self.imageDisplay)
        hbox.addWidget(self.topic_selector)
        hbox.addWidget(self.angle_selector)

        self.angle_selector.turnAngle.connect(self.setAngle)
        self.topic_selector.currentIndexChanged.connect(self._topic_sel_calllback)

    def get_active_topic(self):
        return self.topic_selector.currentData()

    @pyqtSlot(int)
    def setAngle(self, angle):
        print(angle)
        self.angle = angle

    @pyqtSlot(QImage)
    def newSample(self, image):
        if image == None or image.isNull():
            self.imageDisplay.setText("No Image")
            return

        # todo: add scaling of the image
        if self.angle != 0:
            image_rotated = image.transformed(QTransform().rotate(self.angle), Qt.SmoothTransformation)
        else:
            image_rotated = image

        pixmap = QPixmap()
        pixmap = pixmap.fromImage(image_rotated)
        self.imageDisplay.setPixmap(pixmap)

    @pyqtSlot(bool)
    def setControlsVisible(self, visible):
        self.topic_selector.setVisible(visible)
        self.angle_selector.setVisible(visible)

    @pyqtSlot(str)
    def add_feed_entry(self, string):
        if string is not None:
            self.topic_selector.addItem(string)

    def set_feed_list(self, feed_list):
        self.topic_selector.clear()
        for i in feed_list:
            self.topic_selector.addItem(i)

    def _topic_sel_calllback(self, index):
        if index < self.topic_selector.count():
            topic = self.topic_selector.itemText(index)
            self.playTopic.emit(topic)


def cb(string):
    print(string)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = SingleVideoScreen(180)
    ui.show()
    image = QImage("/home/david/rover/3_Main_Inverted.png")
    ui.newSample(image)
    ui.add_feed_entry("String")
    ui.add_feed_entry("String2")
    ui.add_feed_entry("String3")
    ui.add_feed_entry("String4")
    ui.playTopic.connect(cb)
    exit(app.exec_())
