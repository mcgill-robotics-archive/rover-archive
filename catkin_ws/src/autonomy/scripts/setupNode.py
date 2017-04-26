#!/usr/bin/env python

import cv2
import numpy as np
import time
import rospy
import roslib
import sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

rospy.init_node("marker_nav_setup", anonymous=False)


bridge = CvBridge()

# mouse callback function
def mouseCallBack(event,x,y,flags,param):
    global x_center, y_center
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print "Clicked @   X = %r   Y = %r" %(x, y)
        x_center = x
        y_center = y

        getColor(frame)


def getColor(image):
    global x_center, y_center, trackedHSV, trackedBGR

    # Reduce noise, median blur was the best so far
    #blur = cv2.blur(frame,(5,5))
    #blur = cv2.GaussianBlur(frame,(5,5),5)
    blur = cv2.medianBlur(image,5)

    trackedBGR = blur[y_center,x_center]
    print "\nBGR values: B: %r   G: %r   R: %r " %(trackedBGR[0],trackedBGR[1],trackedBGR[2])

    trackedHSV = BGRtoHSV(trackedBGR)

    print "\nHSV values: H: %r   S: %r   V: %r " %(trackedHSV[0],trackedHSV[1],trackedHSV[2])

def BGRtoHSV(bgr_array):

    h,s,v = 0,0,0

    Bp = bgr_array[0]/255.0
    Gp = bgr_array[1]/255.0
    Rp = bgr_array[2]/255.0

    Cmax = float(max(Bp, Gp, Rp))
    Cmin = float(min(Bp, Gp, Rp))
    delta = float(Cmax-Cmin)

    if delta == 0.0:
        h = 0.0
    elif Cmax == Rp:
        h = 60*((Gp-Bp)/delta)
    elif Cmax == Gp:
        h = 60*(((Bp-Rp)/delta)+2)
    else:
        h = 60*(((Rp-Gp)/delta)+4)

    if Cmax == 0.0:
        s = 0.0
    else:
        s = delta/Cmax

    v = Cmax

    # OPENCV works with values of H going from 0 to 180 (not 0 to 360),
    # S and V range from 0 to 255
    return [abs(h/2),s*255,v*255]

#cv2.namedWindow('MarkerFeed')
#cv2.setMouseCallback('MarkerFeed',mouseCallBack)

def callback(data):
    
    #Convert ROS Image message to an openCV image
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    #Add a rectangle to the openCV image
    cv2.imshow('MarkerFeed', frame)

    """
    #Convert the openCV image to a ROS CompressedImage message
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()

    markedPub.publish(msg)


    #Lines below would instead be used if the published Image was uncompressed
    
    try:
        markedPub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    except CvBridgeError as e:
        print(e)
    """
    
    

if __name__ == '__main__':
    incomingFeed = rospy.Subscriber("usb_cam/image_raw", Image, callback, queue_size=10)
 
    """
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
    """
    rospy.spin()
