#!/usr/bin/env python



#from __future__ import print_function

#^^Not sure what this line does, so commented it out^^

import roslib
import numpy as np
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

cap = cv2.VideoCapture(0)

if __name__ == '__main__':

    rospy.init_node("omnicam_capture", anonymous=False)
    pubGRAY = rospy.Publisher("/omnicam_RAW_GRAY", Image, queue_size=1)
    #pubBGR = rospy.Publisher("/omnicam_RAW_BGR", Image, queue_size=1)
    bridge = CvBridge()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # Capture frame-by-frame.
        ret, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   
        # Display the resulting frames (please use rqt instead of imshow).
        #cv2.imshow('Normal Frame', frame)
        #cv2.imshow('Gray Frame', gray)
        
        GRAYros = bridge.cv2_to_imgmsg(gray, "mono8")
        #BGRros = bridge.cv2_to_imgmsg(frame, "bgr8")

        pubGRAY.publish(GRAYros)
        #pubBGR.publish(BGRros)

        rate.sleep()

    # When everything done, release the capture.
    cap.release()
    cv2.destroyAllWindows()
