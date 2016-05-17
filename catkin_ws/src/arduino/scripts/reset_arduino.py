#!/usr/bin/env python

from arduino.srv import *
import rospy

def service_callback(request):
    rospy.loginfo("[Arduino Reset] : Resetting the arduino")

    #pull pin high#
    rospy.Rate(10).sleep()
    #pull pin low again#

    return ResetArduinoResponse(True)


def reset_arduino_server():
    rospy.init_node('reset_arduino')
    s = rospy.Service('reset_arduino_srv', ResetArduino, service_callback)
    rospy.loginfo("[Arduino Reset] : Server ready to process requests")
    rospy.spin()

if __name__ == '__main__':
    reset_arduino_server()