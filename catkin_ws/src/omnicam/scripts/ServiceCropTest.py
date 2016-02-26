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
from omnicam.srv import *

first_shot = True
#Position of sample frame
x,y = 0,0
old_x, old_y, old_h = 0,0,0

sizein_x,sizein_y = 0,0
#Height & Width of window sample
h,w = 0,0

req_x = 0
req_y = 0
req_zoom = 0

rospy.init_node("omnicam_crop", anonymous=False)
cropPub = rospy.Publisher("/omnicam_cropped", Image, queue_size=1)

bridge = CvBridge()

def receive_view_command(request):
    global req_zoom
    global req_x
    global req_y
    req_zoom += request.zoom
    req_y += request.y
    req_x += request.x
    return ControlViewResponse(True)

def constrain(val,min,max):
    if(val < min):
        val = min
    if(val > max):
        val = max
    return val

def specialCrop(feedtocrop,x1,y1,w1,h1):
    global sizein_x, sizein_y

    roi = feedtocrop[sizein_y/2-h1/2:sizein_y/2+h1/2, sizein_x/2-w1/2:sizein_x/2+w1/2]

    if(x1-w1/2 < 0):
        roi[0:roi.shape[0], 0:w1/2-x1] = feedtocrop[y1-h1/2:y1+h1/2, sizein_x+x1-w1/2:sizein_x-1]
        roi[0:roi.shape[0], w1/2-x1:roi.shape[1]] = feedtocrop[y1-h1/2:y1+h1/2, 0:x1+w1/2+1]
    else:
        roi[0:roi.shape[0], 0:w1/2+sizein_x-x1] = feedtocrop[y1-h1/2:y1+h1/2, x1-w1/2:sizein_x-1]
        roi[0:roi.shape[0], w1/2+sizein_x-x1:roi.shape[1]] = feedtocrop[y1-h1/2:y1+h1/2, 0:w1/2+x1-sizein_x+1]

    return roi

def readPos(new_x,new_y,new_h):
    
    global req_x, req_y, req_zoom, sizein_x, sizein_y
    new_w = 0

    if(req_zoom != 0):
        new_h -= req_zoom
        new_h = constrain(new_h, 100, sizein_y)
    new_w = new_h/480.0*640.0

    if(req_x != 0):
        new_x += req_x
    
    if(req_y != 0):
        new_y -= req_y

    new_y = constrain(new_y, new_h/2, sizein_y-new_h/2)

    if(new_x < 0):
        new_x += sizein_x
    if(new_x > sizein_x):
        new_x -= sizein_x

    return (new_x,new_y,new_h,new_w)
    
def crop(img_in):
    
    global first_shot, x, y, old_x, old_y, old_h, h, w, req_x, req_y, req_zoom
    crop_img = img_in[y-h/2:y+h/2, x-w/2:x+w/2]
    global sizein_x, sizein_y
    if (first_shot):
        sizein_x = img_in.shape[1]
        sizein_y = img_in.shape[0]

        x, old_x = img_in.shape[1]/2.0, img_in.shape[1]/2.0
        y, old_y = img_in.shape[0]/2.0, img_in.shape[0]/2.0

        h, old_h = img_in.shape[0], img_in.shape[0]
        w = h/480*640

        first_shot = False
    
    x,y,h,w = readPos(old_x, old_y, h)
   
    #Crop the feed ([startY:endY, startX:endX] format)
    if(x-w/2 < 0 or x+w/2 > sizein_x):
        #Continuous 360 feed!
        crop_img = specialCrop(img_in,x,y,w,h)
    else:
        crop_img = img_in[y-h/2:y+h/2, x-w/2:x+w/2]
    
    #Resize the feed to a 480 x 640 format
    #resized = cv2.resize(crop_img, (640, 480))
    
    old_x = x
    old_y = y
    old_h = h

    
    #Reset values of service commands
    req_x = 0
    req_y = 0
    req_zoom = 0

    return crop_img

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
    service = rospy.Service("crop_control", ControlView, receive_view_command)
    rawGRAY = rospy.Subscriber("/omnicam_unwarp_GRAY", Image, callback, queue_size=1)
    rospy.spin()
