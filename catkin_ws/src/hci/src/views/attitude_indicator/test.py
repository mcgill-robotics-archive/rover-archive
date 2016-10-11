import sys

import time

from PyQt5.QtCore import QThread
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from views.attitude_indicator.attitude import QAttitude
from views.attitude_indicator.compass import QCompass


class Thread(QThread):
    newValue = pyqtSignal(int, name="newValue")

    def __init__(self, parent=None):
        super(Thread, self).__init__(parent)

    def run(self):
        while 1:
            for i in range(-30, 30):
                self.newValue.emit(i)
                time.sleep(0.01)
                # print(i)


class Window(QWidget):
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)

        vbox = QVBoxLayout()
        self.alt = QAttitude(self)
        self.comp = QCompass(self)

        vbox.addWidget(self.alt)
        vbox.addWidget(self.comp)
        self.setLayout(vbox)

        thread = Thread(self)
        thread.newValue.connect(self.alt.setRoll)
        thread.newValue.connect(self.alt.setPitch)
        thread.newValue.connect(self.comp.setYaw)
        thread.start()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Window()
    window.show()
    exit(app.exec())
