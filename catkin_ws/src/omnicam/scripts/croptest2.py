#!/usr/bin/env python

#from __future__ import print_function
import sys
import time
import numpy as np
import cv2
import roslib
import rospy
#from stdmsgs.msg import String
from cv_bridge import CvBridge, CvBridge
from sensor_msgs.msg import Image

first_shot = True
x,y = 0,0
old_x, old_y, = 0,0
h,w = 0,0

rospy.init_node("omnicam_crop", anonymous=False)
cropPub = rospy.Publisher("/omnicam_cropped", Image, queue_size=1)

bridge = CvBridge()

def readPos(current_x,current_y):
    
    return (current_x,current_y)

def crop(img_in):
    
    global first_shot, x, y, old_x, old_y, h, w
    
    if (first_shot):
        x, old_x = img_in.shape[1]/2.0, img_in.shape[1]/2.0
        y, old_y = img_in.shape[0]/2.0, img_in.shape[0]/2.0

        h = 100
        w = h/480*640

        first_shot = False
    
    x,y = readPos(old_x, old_y)
    w = h/480.0*640.0
    print x+h/480.0*640.0
    #Crop the feed ([startY:endY, startX:endX] format)
    crop_img = img_in[y-h/2:y+h/2, x-w/2:x+w/2]
    
    
    #Resize the feed to a 480 x 640 format
    resized = cv2.resize(crop_img, (640, 480))
    
    old_x = x
    old_y = y
    
    return resized

def callback(data):
    try:
        cv_incoming = bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
        print(e)

    cropped = crop(cv_incoming)

    try:
        cropPub.publish(bridge.cv2_to_imgmsg(cropped, "mono8"))
    except CvBridgeError as e:
        print(e)

if __name__ == '__main__':
    rawGRAY = rospy.Subscriber("/omnicam_RAW_GRAY", Image, callback, queue_size=1)
    rospy.spin()
