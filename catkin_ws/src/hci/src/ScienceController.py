
import rospy
from PyQt4 import QtCore
from std_msgs.msg import Int16
from arduino.msg import LimitSwitchScience


class ScienceController(QtCore.QObject):
    
    limit_switch_up_on = QtCore.pyqtSignal()
    limit_switch_down_on = QtCore.pyqtSignal()
    limit_switch_up_off = QtCore.pyqtSignal()
    limit_switch_down_off = QtCore.pyqtSignal()

    def __init__(self):

        QtCore.QObject.__init__(self)

        self.height_publisher = rospy.Publisher("/science/auger_position", Int16, queue_size=1)
        self.drill_publisher = rospy.Publisher("/science/auger_velocity", Int16, queue_size=1)
        self.soil_gate_publisher = rospy.Publisher("/science/soil_servo_position", Int16, queue_size=1)
        self.rock_gate_publisher = rospy.Publisher("/science/rock_servo_position", Int16, queue_size=1)
        self.limit_switch_science_status = LimitSwitchScience()
        rospy.Subscriber("/science/limit_switch", LimitSwitchScience, self.limit_switch_callback, queue_size=1)

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

    def open_rock_gate(self):
        mes = Int16()
        mes.data = 1
        self.rock_gate_publisher.publish(mes)

    def close_rock_gate(self):
        mes = Int16()
        mes.data = 0
        self.rock_gate_publisher.publish(mes)

    def open_soil_gate(self):
        mes = Int16()
        mes.data = 1
        self.soil_gate_publisher.publish(mes)

    def close_soil_gate(self):
        mes = Int16()
        mes.data = 0
        self.soil_gate_publisher.publish(mes)

    def limit_switch_callback(self, msg):
        self.limit_switch_science_status = msg

        if msg.limit_switch_up:
            self.limit_switch_up_on.emit()
        else:
            self.limit_switch_up_off.emit()

        if msg.limit_switch_down:
            self.limit_switch_down_on.emit()
        else:
            self.limit_switch_down_off.emit()
