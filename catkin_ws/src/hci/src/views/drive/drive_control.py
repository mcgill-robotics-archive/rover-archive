"""!@brief Graphical widget to select and control steering configuration

This modules includes the gui class and the Data Object passed by the signal
"""

import sys
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QCheckBox
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QRadioButton
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget


class DriveSettings(object):
    """!@brief Data class used by the signal in the SteeringMode class

    Basically a dictionary, contains status variables for the three
    steering configurations supported
    """

    def __init__(self, motor_enable=False, ackerman_steering=False, point_steering=False, translatory_steering=False):
        """!@brief Constructor initializes values

        @param self Python object pointer
        @param motor_enable Allow motors to turn
        @param ackerman_steering Car like steering
        @param point_steering No linear velocity
        @param translatory_steering No angular velocity
        """

        self.motor_enable = motor_enable
        self.ackerman_steering = ackerman_steering
        self.point_steering = point_steering
        self.translatory_steering = translatory_steering


class SteeringMode(QWidget):
    """!@brief Steering selection window declaration

    Allow selection of the control mode and enabling the motors.
    Interface to the rest of the world is done by signaling when a value is
    changed using the driveSettingsChanged signal.
    """

    ## Signals a gui event changed the current control mode
    driveSettingsChanged = pyqtSignal(DriveSettings, name="changeDriveSettings")

    def __init__(self, parent=None):
        """!@brief Constructor places widgets on screen and connects the
        button signals.

        Places radio buttons with each steering configuration available. When
        clicked, the button triggers the signal with the proper updated
        structure.

        @param self Python object pointer
        @param parent QWidget object hierarchy
        """

        super(SteeringMode, self).__init__(parent)

        self._drive_settings = DriveSettings()

        vbox1 = QVBoxLayout()
        vbox1.setContentsMargins(0, 0, 0, 0)
        hbox1 = QHBoxLayout()
        hbox1.setContentsMargins(0, 0, 0, 0)
        grid1 = QGridLayout()
        grid1.setContentsMargins(0, 0, 0, 0)

        self._ackerman = QRadioButton(self)
        self._ackerman.setText("Ackerman Steering")
        self._pointsteer = QRadioButton(self)
        self._pointsteer.setText("Point Steering")
        self._translate = QRadioButton(self)
        self._translate.setText("Translation")

        self._enable = QCheckBox()
        self._enable.setText("Enable Motors")

        vbox1.addWidget(self._ackerman)
        vbox1.addWidget(self._pointsteer)
        vbox1.addWidget(self._translate)
        hbox1.addWidget(self._enable)
        hbox1.addItem(vbox1)

        self.setLayout(hbox1)

        self._enable.toggled.connect(self._handle_enable)
        self._ackerman.clicked.connect(self._handle_steering_mode)
        self._pointsteer.clicked.connect(self._handle_steering_mode)
        self._translate.clicked.connect(self._handle_steering_mode)

    def _handle_enable(self):
        """!@brief Enable button callback

        Update the status structure with the current status of the enable
        checkbox and emit signal

        @param self Python object pointer
        """

        self._drive_settings.motor_enable = self._enable.isChecked()
        self.driveSettingsChanged.emit(self._drive_settings)

    def _handle_steering_mode(self):
        """!@brief Radio buttons callback

        Handle changes in radio buttons. Update the status structure and emit
        signal.

        @param self Python object pointer
        """

        if self._ackerman.isChecked():
            self._drive_settings.ackerman_steering = True
            self._drive_settings.point_steering = False
            self._drive_settings.translatory_steering = False
        elif self._pointsteer.isChecked():
            self._drive_settings.ackerman_steering = False
            self._drive_settings.point_steering = True
            self._drive_settings.translatory_steering = False
        elif self._translate.isChecked():
            self._drive_settings.ackerman_steering = False
            self._drive_settings.point_steering = False
            self._drive_settings.translatory_steering = True

        self.driveSettingsChanged.emit(self._drive_settings)

    def update_motor_enable(self, motor_enable=False):
        """!@brief External update of the checkbox.

        @param self Python object pointer
        @param motor_enable Enable status
        """

        self._drive_settings.motor_enable = motor_enable
        self._enable.setChecked(motor_enable)
        self.driveSettingsChanged.emit(self._drive_settings)

    def update_steering(self, ackerman_steering=False, point_steering=False, translatory_steering=False):
        """!@brief External updates of the steering configuration buttons

        @param self Python object pointer
        @param ackerman_steering Car like steering
        @param point_steering No linear velocity
        @param translatory_steering No angular velocity
        """

        self._ackerman.setChecked(ackerman_steering)
        self._pointsteer.setChecked(point_steering)
        self._translate.setChecked(translatory_steering)

        self._drive_settings.ackerman_steering = ackerman_steering
        self._drive_settings.point_steering = point_steering
        self._drive_settings.translatory_steering = translatory_steering

        self.driveSettingsChanged.emit(self._drive_settings)

    @pyqtSlot(DriveSettings)
    def overrideStatus(self, status):
        """!@brief Slot for updating the entire status structure.
        @param self Python object pointer
        @param status DriveSettings structure to replace
        """

        self._enable.setChecked(status.motor_enable)
        self.update_steering(status.ackerman_steering, status.point_steering, status.translatory_steering)


if __name__ == "__main__":

    class DriveWindowTest(QWidget):
        """!@brief Test window that uses SteeringMode
        """

        def __init__(self):
            """!@brief Constructor shows window
            @param self Python object pointer

            """

            super(DriveWindowTest, self).__init__()
            self.ui = SteeringMode(self)
            self.ui.show()
            self.ui.driveSettingsChanged.connect(self.print_status)

        @pyqtSlot(DriveSettings)
        def print_status(self, drive_setting):
            """!@brief Print status structure
            @param self Python object pointer
            @param drive_setting The struc to print

            """

            if drive_setting.ackerman_steering:
                stering = "ackerman"
            elif drive_setting.point_steering:
                stering = "point"
            elif drive_setting.translatory_steering:
                stering = "translatory"
            else:
                stering = "none"

            print("Status: enable {0}, steering: {1}".format(drive_setting.motor_enable, stering))


    app = QApplication(sys.argv)
    ui = DriveWindowTest()
    ui.show()
    ui.ui.update_motor_enable(True)
    ui.ui.update_steering(translatory_steering=True)
    sys.exit(app.exec_())
