
import rospy
from arduino.srv import *
from std_msgs.msg import Float32


class ScienceController(object):
    def __init__(self):
        self.height_publisher = rospy.Publisher("/augur_height", Float32, queue_size=1)
        try:
            rospy.wait_for_service("/drill", timeout=2)
            self.drill_service = rospy.ServiceProxy("/drill", drill)

        except rospy.ROSException:
            rospy.logerr("Could not acquire the drill service")
            self.drill_service = None

    def publish_auger_height(self, speed):
        mes = Float32()
        mes.data = speed
        self.height_publisher.publish(mes)

    def deactivate_drill(self):
        rospy.logerr("Try to deactivate")
        if self.drill_service:
            try:
                resp = self.drill_service(False)
                return resp.activated

            except rospy.ROSException:
                rospy.logerr("Failed to change drill")
                return True
        else:
            return False

    def activate_drill(self):
        rospy.logerr("Try to activate")

        if self.drill_service:
            try:
                resp = self.drill_service(True)
                return resp.activated

            except rospy.ROSException:
                rospy.logerr("Failed to change drill")
                return False
        else:
            return True
