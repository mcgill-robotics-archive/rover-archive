import rospy
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from drive_control.msg import DriveCommand
from geometry_msgs.msg import Twist
from rover_common.msg import MotorStatus

from controller.joystick.joystick_base import JoystickBase
from model.joystick_data import JoystickData
from views.drive.drive_control import DriveSettings
from views.drive.wheel_status import WheelStatusStruct

zero = 1e-8


class DriveController(JoystickBase):
    """!@brief Control drive related information and communications

    General logic elements related to the drive modules, including the ROS
    publisher and subscriber to the drive controller and motor controller
    feedback nodes.
    """

    ## Emitted when a new values of wheel status is received
    wheelStatusUpdate = pyqtSignal(WheelStatusStruct)

    ## Emitted to update the displayed steering configuration selection
    forceControlsUpdate = pyqtSignal(DriveSettings)

    class Mode:
        HCI = 1
        Autonomous = 2

        def __init__(self):
            pass

    def __init__(self, parent=None):
        """!@brief Constructor creates the publisher and subscriber

        @param self Python object pointer
        @param parent QObject parent in Qt hierarchy
        """

        super(DriveController, self).__init__(parent)

        self.mode = DriveController.Mode.HCI
        self._command_publisher = rospy.Publisher("/drive_command", DriveCommand, queue_size=1)
        self._status_subscriber = rospy.Subscriber('/motor_status', MotorStatus, self._motor_status, queue_size=1)
        self.autonomousSubscriber = rospy.Subscriber('/cmd_vel', DriveCommand, self._filter_autonomous_nav, queue_size=1)
        self._settings = DriveSettings()
        self._command = Twist()
        self.autonomous_message = DriveCommand()

    @pyqtSlot(JoystickData)
    def handle_joystick_data(self, data):
        """!@brief Implementation of the base class interface

        Receives the joystick data and updates the internal data structure.
        Finally publish the control commands to the drive control node.

        @param self Python object pointer
        @param data JoystickData structure containing all data elements.
        """
        emit_signal = False
        if self.mode == DriveController.Mode.HCI:
            self._command.linear.x = data.a2
            self._command.angular.z = data.a1
            if self._settings.motor_enable != data.b1:
                self._settings.motor_enable = data.b1
                emit_signal = True

            if data.b2:
                self._settings.ackerman_steering = True
                self._settings.point_steering = False
                self._settings.translatory_steering = False
                emit_signal = True

            elif data.b3:
                self._settings.ackerman_steering = False
                self._settings.point_steering = True
                self._settings.translatory_steering = False
                emit_signal = True

            elif data.b4:
                self._settings.ackerman_steering = False
                self._settings.point_steering = False
                self._settings.translatory_steering = True
                emit_signal = True

            if emit_signal:
                self.forceControlsUpdate.emit(self._settings)

        elif self.mode == DriveController.Mode.Autonomous:
            self._command = self.autonomous_message.velocity_command
            if abs(self.autonomous_message.velocity_command.linear.x) < zero:
                self._settings.ackerman_steering = False
                self._settings.point_steering = True
            else:
                self._settings.ackerman_steering = True
                self._settings.point_steering = False

            if abs(self.autonomous_message.velocity_command.linear.x) > zero or abs(self.autonomous_message.velocity_command.angular.z) > zero:
                self._settings.motor_enable = True
            else:
                self._settings.motor_enable = False

            self.forceControlsUpdate.emit(self._settings)

        self._publish()

    def _motor_status(self, status):
        wheel_status = WheelStatusStruct(status.fl, status.ml, status.bl, status.fr, status.mr, status.br)
        self.wheelStatusUpdate.emit(wheel_status)

    @pyqtSlot(DriveSettings)
    def setDriveSetting(self, settings):
        """!@brief Slot to update the control setting for next messages

        @param self Python object pointer
        @param settings DriveSettings object with control values
        """

        self._settings = settings

    @pyqtSlot()
    def activateHCIMode(self):
        self.mode = DriveController.Mode.HCI
        print("HCI Mode")

    @pyqtSlot()
    def activateAutonomousMode(self):
        self.mode = DriveController.Mode.Autonomous
        print("Autonomous Mode")

    def _filter_autonomous_nav(self, message):
        self.autonomous_message = message

    def _publish(self):
        msg = DriveCommand()
        msg.motion_ackerman = self._settings.ackerman_steering
        msg.motion_pointsteer = self._settings.point_steering
        msg.motion_translatory = self._settings.translatory_steering
        msg.motion_enable = self._settings.motor_enable

        msg.velocity_command = self._command

        self._command_publisher.publish(msg)
