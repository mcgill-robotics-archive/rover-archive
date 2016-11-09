import math
import rospy
import tf
from PyQt5.QtCore import QObject
from PyQt5.QtCore import pyqtSignal
from ahrs.msg import AhrsStdMsg
from geometry_msgs.msg import Pose


class NavigationController(QObject):
    updatePitch = pyqtSignal(float)
    updateRoll = pyqtSignal(float)
    updateYaw = pyqtSignal(float)
    updateAttitude = pyqtSignal(Pose)

    def __init__(self, parent=QObject):
        super(NavigationController, self).__init__(parent)

        self._attitude_subscriber = rospy.Subscriber("/ahrs_status", AhrsStdMsg, self._handle_ahrs_message)

    def _handle_ahrs_message(self, message):
        print("New message")
        pose = message.pose.pose

        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.updateRoll.emit(math.degrees(euler[0]))
        self.updatePitch.emit(math.degrees(euler[1]))
        self.updateYaw.emit(math.degrees(euler[2]))
        self.updateAttitude.emit(pose)
