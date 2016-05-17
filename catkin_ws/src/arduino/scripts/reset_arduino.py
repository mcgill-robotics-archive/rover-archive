#!/usr/bin/env python

from arduino.srv import *
import rospy
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error, could not import the GPIO library")
    exit(1)

class ResetArduinoServer(object):
    def __init__(self, output_pin):
        #todo: change channel to proper pin (maybe a ros param)
        self.channel = 7        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.channel, GPIO.OUT, initial=GPIO.LOW)
        s = rospy.Service('reset_arduino_srv', ResetArduino, service_callback)
        rospy.loginfo("[Arduino Reset] : Server ready to process requests")
        rospy.spin()
    
    def service_callback(self, request):
        rospy.loginfo("[Arduino Reset] : Resetting the arduino")

        #pull pin high#
        GPIO.output(self.channel, GPIO.HIGH)
        rospy.Rate(10).sleep()
        #pull pin low again#
        GPIO.output(self.channel, GPIO.LOW)

        return ResetArduinoResponse(True)

    def __del__(self):
        GPIO.cleanup()

if __name__ == '__main__':
    rospy.init_node('reset_arduino')
    ResetArduinoServer()

