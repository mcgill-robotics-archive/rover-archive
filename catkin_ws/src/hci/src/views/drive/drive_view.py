"""!@brief Main Drive view, single class to instantiate and interface to
in order to use the drive components.
"""

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from views.drive.drive_control import SteeringMode, DriveSettings
from views.drive.wheel_status import WheelStatus, WheelStatusStruct


class DriveView(QWidget):
    """!@brief Main display class for drive components

    Includes the WheelStatus and SteeringMode classes and forwards signals and
    slots properly. All interfacing to underlying classes is forbidden.
    """

    ## Signals the control structure was updated and passes the newest value
    controlsUpdated = pyqtSignal(DriveSettings)

    def __init__(self, parent=None):
        """!@brief Constructor of the view.

        Creates the elements and lays them out. Also creates the forwarding
        signals and wraps the update methods.

        @param self Python object pointer
        @param parent The QWidget parent in Qt hierarchy
        """

        super(DriveView, self).__init__(parent)

        self._control_view = SteeringMode(self)
        self._wheel_status = WheelStatus(self)

        self._layout = QVBoxLayout()
        self._layout.addWidget(self._control_view)
        self._layout.addWidget(self._wheel_status)

        self.setLayout(self._layout)

        self._control_view.driveSettingsChanged.connect(self._forwardControls)

    @pyqtSlot(int)
    def show_motor_enable(self, motor_enable):
        self._control_view.showEnableMotor(motor_enable)

    @pyqtSlot(WheelStatusStruct)
    def updateMotorStatus(self, status):
        """!@brief Forward to WheelStatus::update_motor_status

        @param self Python object pointer
        @param status The status structure
        """
        self._wheel_status.update_motor_status(status)

    @pyqtSlot(DriveSettings)
    def displayDriveSettings(self, settings):
        """!@brief Slot to forward to SteeringMode::overrideStatus
        @param self Python object pointer
        @param settings The new drive settings
        """

        self._control_view.overrideStatus(settings)

    @pyqtSlot(DriveSettings)
    def _forwardControls(self, controls):
        """!@brief Forward the updated control settings signals
        @param self Python object pointer
        @param controls The control struct
        """

        self.controlsUpdated.emit(controls)
