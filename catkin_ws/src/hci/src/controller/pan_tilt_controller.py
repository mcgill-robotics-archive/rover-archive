import rospy
from geometry_msgs.msg import Twist

from controller.joystick.joystick_base import JoystickBase


class PanTiltController(JoystickBase):
    def __init__(self, parent=None):
        super(PanTiltController, self).__init__(parent)

        self.pt_pub = rospy.Publisher("/pan_camera_command", Twist, queue_size=1)

    def handle_joystick_data(self, data):
        message = Twist()
        message.linear.x = data.a1 * 100
        message.linear.y = data.a2
        self.pt_pub.publish(message)
