#!/usr/bin/env python

import roslib

import sys
import rospy
import cv2
import numpy as np
import time
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class waypoint_identifier:

  def __init__(self):
    self.image_pub = rospy.Publisher("marker_feed",Image,queue_size=1)
    self.angle_pub = rospy.Publisher("marker_angle",Float32,queue_size=1)
    self.acquired_pub = rospy.Publisher("marker_acquired",Bool,queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback,queue_size=1)

    self.trackedHSV = [56.3,10.4,196.0]
    self.currentPos = [0,0,0,0] #current x,y,w,h
    self.previousPos = [0,0,0,0] #previous x,y,w,h
    self.previousArea = 0
    self.currentArea = 0
    self.areaTimerON = False
    self.timerStart = time.time()
    self.camFOV = 80 #degrees

  def callback(self,data):
    
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #Establish lower and upper bound of colorspace region of interest
    lower_bound = np.array([self.trackedHSV[0]-5,self.trackedHSV[1]-50,self.trackedHSV[2]-30])
    upper_bound = np.array([self.trackedHSV[0]+10,self.trackedHSV[1]+50,self.trackedHSV[2]+70])

    # Noise filtering (median or Gaussian filter)
    #blur_img = cv2.medianBlur(frame,5)
    blur_img = cv2.GaussianBlur(frame,(5,5),5)

    hsv_img = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lower_bound, upper_bound)

    #Get rid of background noise using erosion and fill in the holes using dilation
    element = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
    eroded = cv2.erode(mask,element, iterations=0)
    dilated = cv2.dilate(eroded,element,iterations=6)


    # Identify the right feature on the dilated image
    img3, contours, hierarchy = cv2.findContours(dilated, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        self.currentArea = cv2.contourArea(contour)
        self.currentPos = cv2.boundingRect(contour)

    #Verify if area doesn't vary for more than 1 sec (means that we have a false positive)
    if self.previousArea == self.currentArea:
        if (not self.areaTimerON):
            self.areaTimer = time.time()
            self.areaTimerON = True
        else:
            self.areaTimeout = (time.time() - self.areaTimer) > 1
    else:
        self.areaTimeout = False
        self.areaTimerON = False

    #Verify if our detection is stable & consistent
    if abs(self.currentPos[0]-self.previousPos[0])<50 and abs(self.currentPos[1]-self.previousPos[1])<50 and abs(self.currentArea-self.previousArea)<(self.currentArea*0.3) and (not self.areaTimeout):
        stable = True
        rectColor = [0,0,255]
    else:
        stable = False
        self.timerStart = time.time()

    # If detection is consistent for 3 seconds, confirm that the marker is detected
    if stable and abs(time.time()-self.timerStart)>3:
        markerLocked = True
        rectColor = [255,0,0]
    else:
        markerLocked = False

    # Draws rectangle around detection (red means preliminary detection, blue means detection confirmed)
    if stable:
        cv2.rectangle(frame, (self.currentPos[0],self.currentPos[1]),(self.currentPos[0]+self.currentPos[3],self.currentPos[1]+self.currentPos[3]), rectColor, 3)

    self.previousPos = self.currentPos
    self.previousArea = self.currentArea

    if markerLocked:
        #Output marker angular position from the center of the feed
        #Center is 0 degrees, left is +ve, right is -ve
        offset = (frame.shape[1]/2.0-(self.currentPos[0]+self.currentPos[2]/2.0))/(frame.shape[1]/2.0)*(self.camFOV/2)
        stopCommand = self.currentArea > 8000
        print offset, stopCommand
    else:
        stopCommand = False
        offset = 0.0

    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
      self.angle_pub.publish(offset)
      self.acquired_pub.publish(markerLocked)
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('marker_navigator', anonymous=True)
  wi = waypoint_identifier()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)