import sys
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QWidget


class JoystickMode(QWidget):
    changeMode = pyqtSignal(str)

    def __init__(self, parent=None):
        super(JoystickMode, self).__init__(parent)

        self.button_list = []
        self.layout = QHBoxLayout()
        self.setLayout(self.layout)

    @pyqtSlot(str)
    def addMode(self, name):
        button=QPushButton(name, self)
        self.layout.addWidget(button)

        def button_callback():
            name = button.text()
            self.changeMode.emit(name)

        button.clicked.connect(button_callback)


def callback(string):
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
