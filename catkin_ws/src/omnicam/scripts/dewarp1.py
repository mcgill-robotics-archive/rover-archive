#!/usr/bin/env python

#from __future__ import print_function
import roslib
import sys
import rospy
import cv2
#from stdmsgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

# build the mapping
def buildMap(Wd,Hd,R1,R2,Cx,Cy):
    map_x = np.zeros((Hd,Wd),np.float32)
    map_y = np.zeros((Hd,Wd),np.float32)
    for y in range(0,int(Hd-1)):
        for x in range(0,int(Wd-1)):
            r = (float(y)/float(Hd))*(R2-R1)+R1
            theta = (float(x)/float(Wd))*2.0*np.pi
            xS = Cx+r*np.sin(theta)
            yS = Cy+r*np.cos(theta)
            map_x.itemset((y,x),int(xS))
            map_y.itemset((y,x),int(yS))
        
    return map_x, map_y

# do the unwarping 
def unwarp(img,xmap,ymap):
    output = cv2.remap(img,xmap,ymap,cv2.INTER_LINEAR)
    return output

#Cx, Cy, R1x, R1y, R2x, R2y can be determined using dewarp_set.py
# 0 = xc yc
# 1 = r1
# 2 = r2
# center of the "donut"    
Cx = 303
Cy = 265
# Inner donut radius
R1x = 364
R1y = 268
R1 = R1x-Cx
# outer donut radius
R2x = 542
R2y = 266
R2 = R2x-Cx
# our input and output image siZes
Wd = 2.0*((R2+R1)/2)*np.pi
Hd = (R2-R1)
# build the pixel map, this could be sped up
print "BUILDING MAP!"

xmap,ymap = buildMap(Wd,Hd,R1,R2,Cx,Cy)
print "MAP DONE!"

rospy.init_node("omnicam_unwarp", anonymous=False)
unwarpGRAY = rospy.Publisher("/omnicam_unwarp_GRAY", Image, queue_size=1)
#unwarpBGR = rospy.Publisher("/omnicam_unwarp_BGR", Image, queue_size=1)

#NOTE: IF WANT TO USE BGR FEED, COMMENT OUT GRAY FEED STUFF AND REPLACE
#the "mono8" encoding by "bgr8" and replace unwarpGRAY.publish... by
# unwarpBGR.publish... . Or literally create another callback
#function with an "bgr8" encoding and unwarpBGR.publish... .

bridge = CvBridge()

def callback(data):
    try:
        cv_incoming = bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
        print(e)

    result = unwarp(cv_incoming, xmap, ymap)

    try:
        unwarpGRAY.publish(bridge.cv2_to_imgmsg(result, "mono8"))
    except CvBridgeError as e:
        print(e)
    

if __name__ == '__main__':
    rawGRAY = rospy.Subscriber("/omnicam_RAW_GRAY", Image, callback, queue_size=1)
    #rawBGR = rospy.Subscriber("/omnicam_RAW_BGR", Image, callback, queue_size=1)
    
    rospy.spin()
