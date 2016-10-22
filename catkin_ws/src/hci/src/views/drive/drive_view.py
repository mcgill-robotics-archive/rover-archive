from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from views.drive.drive_control import SteeringMode, DriveSettings
from views.drive.wheel_status import WheelStatus, WheelStatusStruct


class DriveView(QWidget):
    controlsUpdated = pyqtSignal(DriveSettings)

    def __init__(self, parent=None):
        super(DriveView, self).__init__(parent)

        self.control_view = SteeringMode(self)
        self.wheel_status = WheelStatus(self)

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.control_view)
        self.layout.addWidget(self.wheel_status)

        self.setLayout(self.layout)

        self.control_view.driveSettingsChanged.connect(self._forwardControls)


    @pyqtSlot(WheelStatusStruct)
    def updateMotorStatus(self, status):
        self.wheel_status.update_motor_status(status)

    @pyqtSlot(DriveSettings)
    def displayDriveSettings(self, settings):
        self.control_view.overrideStatus(settings)

    @pyqtSlot(DriveSettings)
    def _forwardControls(self, controls):
        self.controlsUpdated.emit(controls)