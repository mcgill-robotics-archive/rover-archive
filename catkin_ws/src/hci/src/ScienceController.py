
import rospy
from std_msgs.msg import Int16


class ScienceController(object):
    def __init__(self):
        self.height_publisher = rospy.Publisher("/serial_node/auger_position", Int16, queue_size=1)
        self.drill_publisher = rospy.Publisher("/serial_node/auger_velocity", Int16, queue_size=1)
        self.gate_publisher = rospy.Publisher("/serial_node/gate_position", Int16, queue_size=1)
        self.thermo_publisher = rospy.Publisher("/serial_node/temp_prob_position", Int16, queue_size=1)

    def publish_auger_height(self, speed):
        mes = Int16()
        mes.data = speed * 255
        self.height_publisher.publish(mes)

    def deactivate_drill(self):
        rospy.loginfo("Try to deactivate")
        mes = Int16()
        mes.data = 0
        self.drill_publisher.publish(mes)
        return False

    def activate_drill(self):
        rospy.loginfo("Try to activate")
        mes = Int16()
        mes.data = -255
        self.drill_publisher.publish(mes)
        return True

    def move_gate(self, speed):
        mes = Int16()
        mes.data = speed * 5
        self.gate_publisher.publish(mes)

    def move_thermocouple(self, speed):
        mes = Int16()
        if speed > 0:
            mes.data = 50
        else:
            mes.data = -75

        self.thermo_publisher.publish(mes)
