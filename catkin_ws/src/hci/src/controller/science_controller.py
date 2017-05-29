import rospy
from PyQt5.QtCore import pyqtSignal
from std_msgs.msg import Int32

from controller.joystick.joystick_base import JoystickBase


class ScienceController(JoystickBase):
    carriageEncoderUpdate = pyqtSignal(int)
    drillEncoderUpdate = pyqtSignal(int)
    drillSpeedUpdate = pyqtSignal(int)
    carriagePositionUpdate = pyqtSignal(int)

    def __init__(self, parent=None):
        super(ScienceController, self).__init__(parent)

        self.drill_pub = rospy.Publisher("/science/motor_drill", Int32, queue_size=1)
        self.probe_pub = rospy.Publisher("/science/probe", Int32, queue_size=1)
        self.carri_pub = rospy.Publisher("/science/motor_carriage", Int32, queue_size=1)
        self.inc1_sub = rospy.Subscriber("/science/inc_carriage", Int32, self.inc1_cb)
        self.inc2_sub = rospy.Subscriber("/science/inc_drill", Int32, self.inc2_cb)

        self.carriage_message = Int32()
        self.drill_message = Int32()

    def handle_joystick_data(self, data):
        if data.b1:
            self.carriage_message.data = data.a2 * -100
        if data.hat_top:
            self.carriage_message.data += 1
        if data.hat_down:
            self.carriage_message.data -= 1

        self.carri_pub.publish(self.carriage_message)

        self.drill_message.data = data.a4 * -100
        self.drill_pub.publish(self.drill_message)

        self.drillSpeedUpdate.emit(self.drill_message.data)
        self.carriagePositionUpdate.emit(self.carriage_message.data)


    def inc1_cb(self, message):
        self.carriageEncoderUpdate.emit(message.data)

    def inc2_cb(self, message):
        self.drillEncoderUpdate.emit(message.data)
