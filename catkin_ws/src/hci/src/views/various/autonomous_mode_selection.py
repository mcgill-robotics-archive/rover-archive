import sys

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QFrame
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QRadioButton
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget


class AutonomousModeSelection(QWidget):
    activateHCIMode = pyqtSignal()
    activateAutonomousMode = pyqtSignal()

    def __init__(self, parent=None):
        super(AutonomousModeSelection, self).__init__(parent)
        vb1 = QVBoxLayout(self)
        hb1 = QHBoxLayout()

        title_label = QLabel("Navigation Mode")
        self.hci_button = QRadioButton("HCI")
        self.autonomous_button = QRadioButton("Autonomous")
        self.hci_button = QRadioButton("HCI")
        line_1 = QFrame(self)
        line_1.setFrameShape(QFrame.HLine)
        line_1.setFrameShadow(QFrame.Sunken)

        vb1.addWidget(title_label)
        vb1.addLayout(hb1)
        hb1.addWidget(self.hci_button)
        hb1.addWidget(self.autonomous_button)
        vb1.addWidget(line_1)

        self.hci_button.clicked.connect(self.hci_callback)
        self.autonomous_button.clicked.connect(self.autonomous_callback)

        self.hci_button.click()

    def hci_callback(self):
        self.activateHCIMode.emit()

    def autonomous_callback(self):
        self.activateAutonomousMode.emit()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    ui = AutonomousModeSelection()
    ui.show()
    sys.exit(app.exec_())
