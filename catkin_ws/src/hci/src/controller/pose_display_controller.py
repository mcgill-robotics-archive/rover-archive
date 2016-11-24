import rospy
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import QObject
from geometry_msgs.msg import Pose


class PoseDisplayController(QObject):
    """!@brief Communicate the current orientation of the robot
    """

    poseStatusUpdate = pyqtSignal(Pose)

    def __init__(self, parent=None):
        """!@brief Constructor creates the publisher and subscriber

        @param self Python object pointer
        @param parent QObject oarent in Qt hierachy
        """

        super(PoseDisplayController, self).__init__(parent)

        self._status_subscriber = rospy.Subscriber('/pose_status', Pose, self._pose_status,
                                                   queue_size=1)

    def _pose_status(self, status):
        self.poseStatusUpdate.emit(status)
