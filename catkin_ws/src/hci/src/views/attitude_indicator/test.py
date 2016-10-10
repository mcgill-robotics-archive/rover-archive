import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from views.attitude_indicator.attitude import QAttitude
from views.attitude_indicator.compass import QCompass


class Window(QWidget):
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)

        vbox = QVBoxLayout()
        alt = QAttitude(self)
        comp = QCompass(self)

        vbox.addWidget(alt)
        vbox.addWidget(comp)
        self.setLayout(vbox)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Window()
    window.show()
    exit(app.exec())
