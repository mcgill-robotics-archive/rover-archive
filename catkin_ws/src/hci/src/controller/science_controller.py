import rospy
from PyQt5.QtCore import pyqtSignal
from std_msgs.msg import Int32

from controller.joystick.joystick_base import JoystickBase


class ScienceController(JoystickBase):
    carriageEncoderUpdate = pyqtSignal(int)
    drillEncoderUpdate = pyqtSignal(int)
    drillSpeedUpdate = pyqtSignal(int)
    probeSpeedUpdate = pyqtSignal(int)
    carriagePositionUpdate = pyqtSignal(int)
    windUpdate = pyqtSignal(int)
    humidityUpdate = pyqtSignal(int)
    temperatureUpdate = pyqtSignal(int)

    def __init__(self, parent=None):
        super(ScienceController, self).__init__(parent)

        self.drill_pub = rospy.Publisher("/sampling/motor_drill", Int32, queue_size=1)
        self.probe_pub = rospy.Publisher("/sampling/probe", Int32, queue_size=1)
        self.carri_pub = rospy.Publisher("/sampling/motor_carriage", Int32, queue_size=1)
        self.inc1_sub = rospy.Subscriber("/sampling/inc_carriage", Int32, self.inc1_cb)
        self.inc2_sub = rospy.Subscriber("/sampling/inc_drill", Int32, self.inc2_cb)
        self.temp_sub = rospy.Subscriber("/probe_temperature", Int32, self.temp_cb)
        self.wind_sub = rospy.Subscriber("/probe_wind", Int32, self.wind_cb)
        self.humi_sub = rospy.Subscriber("/probe_humidity", Int32, self.humi_cb)

        self.carriage_message = Int32()
        self.drill_message = Int32()
        self.probe_message = Int32()

    def handle_joystick_data(self, data):
        if data.b1:
            self.carriage_message.data = data.a2 * -4000

        if data.b3:
            self.probe_message.data = 80
        elif data.b4:
            self.probe_message.data = 20
        else:
            self.probe_message.data = 0

        if data.hat_top:
            self.carriage_message.data += 1
        if data.hat_down:
            self.carriage_message.data -= 1

        self.drill_message.data = data.a4 * -4000

        self.carri_pub.publish(self.carriage_message)
        self.drill_pub.publish(self.drill_message)
        self.probe_pub.publish(self.probe_message)

        self.drillSpeedUpdate.emit(self.drill_message.data)
        self.carriagePositionUpdate.emit(self.carriage_message.data)
        self.probeSpeedUpdate.emit(self.probe_message.data)


    def inc1_cb(self, message):
        self.carriageEncoderUpdate.emit(message.data)

    def inc2_cb(self, message):
        self.drillEncoderUpdate.emit(message.data)

    def temp_cb(self, message):
        self.temperatureUpdate.emit(message.data)

    def wind_cb(self, message):
        self.windUpdate.emit(message.data)

    def humi_cb(self, message):
        self.humidityUpdate.emit(message.data)
