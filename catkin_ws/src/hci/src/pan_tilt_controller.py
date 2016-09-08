from PyQt4 import QtCore
import rospy
from geometry_msgs.msg import Twist


class PanTiltController(QtCore.QObject):
    def __init__(self):
        QtCore.QObject.__init__(self)
        self.publisher = rospy.Publisher("/pan_camera_command", Twist, queue_size=1)

    def publish_pan_tilt(self, pan_speed, tilt_speed):
        mes = Twist()
        mes.linear.x = pan_speed * 150
        mes.linear.y = tilt_speed * 2
        self.publisher.publish(mes)
