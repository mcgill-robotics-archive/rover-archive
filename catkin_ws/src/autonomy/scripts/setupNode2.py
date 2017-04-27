#!/usr/bin/env python

import roslib
import numpy as np
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from autonomy.msg import hsvBounds
from cv_bridge import CvBridge, CvBridgeError


class marker_setup:

  def __init__(self):
    #
    
    self.hsv_pub = rospy.Publisher("hsvBounds",hsvBounds, queue_size = 10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback)
    
    self.x_center = 0
    self.y_center = 0
    self.trackedBGR = [0.0,0.0,0.0]
    self.trackedHSV = [0.0,0.0,0.0]

  def mouseCallBack(self,event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print "Clicked @   X = %r   Y = %r" %(x, y)
        self.x_center = x
        self.y_center = y

        self.getColor(self.cv_image)

  def getColor(self,image):
    
    # Reduce noise, median blur was the best so far
    #blur = cv2.blur(frame,(5,5))
    #blur = cv2.GaussianBlur(frame,(5,5),5)
    blur = cv2.medianBlur(self.cv_image,5)

    self.trackedBGR = blur[self.y_center,self.x_center]
    print "\nBGR values: B: %r   G: %r   R: %r " %(self.trackedBGR[0],self.trackedBGR[1],self.trackedBGR[2])

    self.trackedHSV = self.BGRtoHSV(self.trackedBGR)

    print "\nHSV values: H: %r   S: %r   V: %r " %(self.trackedHSV[0],self.trackedHSV[1],self.trackedHSV[2])

  def BGRtoHSV(self,bgr_array):

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


  def callback(self,data):
    #r = rospy.Rate(10) #10hz
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    hsvSpace = hsvBounds()
    hsvSpace.lower = [self.trackedHSV[0]-5,self.trackedHSV[1]-50,self.trackedHSV[2]-30]
    hsvSpace.upper = [self.trackedHSV[0]+10,self.trackedHSV[1]+50,self.trackedHSV[2]+70]

    cv2.namedWindow('MarkerFeed')
    cv2.imshow("MarkerFeed", self.cv_image)
    cv2.setMouseCallback('MarkerFeed',self.mouseCallBack)
    cv2.waitKey(3)

    #while not rospy.is_shutdown():
    self.hsv_pub.publish(hsvSpace)
      #r.sleep()

    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)

def main(args):
  rospy.init_node('marker_nav_setup', anonymous=False)
  ms = marker_setup()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)