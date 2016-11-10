"""!@brief Main navigation controller

Single point of entry for the navigation components.
"""

import math
import rospy
import tf
from PyQt5.QtCore import QObject
from PyQt5.QtCore import pyqtSignal
from ahrs.msg import AhrsStdMsg
from geometry_msgs.msg import Pose


class NavigationController(QObject):
    """!@brief Main navigation controller object."""

    ## Emitted when new pitch value should be updated
    updatePitch = pyqtSignal(float)

    ## Emitted when new roll value should be updated
    updateRoll = pyqtSignal(float)

    ## Emitted when new yaw value should be updated
    updateYaw = pyqtSignal(float)

    ## Emitted when new attitude value should be updated
    updateAttitude = pyqtSignal(Pose)

    def __init__(self, parent=QObject):
        """!@brief Constructor creates the ros subscriber and registers callback

        @param self Python object pointer
        @param parent Qt object hierarchy parent
        """
        super(NavigationController, self).__init__(parent)

        self._attitude_subscriber = rospy.Subscriber("/ahrs_status", AhrsStdMsg, self._handle_ahrs_message)

    def _handle_ahrs_message(self, message):
        """Ros callback function for ahrs values

        Computes pitch, roll and yaw from pose and signals new values

        @param self Python object pointer
        @param message AhrsStdMsg status message
        """
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
