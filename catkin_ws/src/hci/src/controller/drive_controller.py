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


class DriveController(JoystickBase):
    wheelStatusUpdate = pyqtSignal(WheelStatusStruct)
    forceControlsUpdate = pyqtSignal(DriveSettings)

    @pyqtSlot(JoystickData)
    def handle_joystick_data(self, data):
        self.command.linear.x = data.a2
        self.command.angular.z = data.a1
        if self.settings.motor_enable != data.b1:
            self.settings.motor_enable = data.b1
            self.forceControlsUpdate.emit(self.settings)

        self.publish()
        pass

    def __init__(self, parent=None):
        super(DriveController, self).__init__(parent)

        self.command_publisher=rospy.Publisher("/drive_command", DriveCommand, queue_size=1)
        self.status_subscriber = rospy.Subscriber('/motor_status', MotorStatus, self.motor_status, queue_size=1)
        self.settings = DriveSettings()
        self.command = Twist()

    def motor_status(self, status):
        wheel_status = WheelStatusStruct(status.fl, status.ml, status.bl, status.fr, status.mr, status.br)
        self.wheelStatusUpdate.emit(wheel_status)
        pass

    @pyqtSlot(DriveSettings)
    def setDriveSetting(self, settings):
        self.settings = settings

    def publish(self):
        msg = DriveCommand()
        msg.motion_ackerman = self.settings.ackerman_steering
        msg.motion_pointsteer = self.settings.point_steering
        msg.motion_translatory = self.settings.translatory_steering
        msg.motion_enable = self.settings.motor_enable

        msg.velocity_command = self.command

        self.command_publisher.publish(msg)
