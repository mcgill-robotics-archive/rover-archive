#!/usr/bin/env python

import cv2
import numpy as np
import time
import rospy
import roslib
import sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

rospy.init_node("waypoint_navigator", anonymous=False)

#Need an Image message and CompressedImage message of the same feed in
#order to be able to display it on rqt or any other image viewer.
#Topic names should follow a similar pattern to below:
# /myImageTopicName  and /myImageTopicName/compressed
uncompPub = rospy.Publisher("waypoint_feed/image_raw", Image, queue_size = 1)
compPub = rospy.Publisher("waypoint_feed/image_raw/compressed", CompressedImage, queue_size=10)

bridge = CvBridge()

def callback(data):
    
    #Convert ROS Image message to an openCV image
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    #Add a rectangle to the openCV image
    cv2.rectangle(frame,(0,0),(100,100),(0,0,255),4)

    #CompressedImage message publisher
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()

    compPub.publish(msg)


    #Image publisher (uncompressed)
    try:
        uncompPub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    except CvBridgeError as e:
        print(e)
    

if __name__ == '__main__':
    incomingFeed = rospy.Subscriber("usb_cam/image_raw", Image, callback, queue_size=10)
    rospy.spin()
