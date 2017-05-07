#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
import time
import math
from autonomy.msg import HSVbounds, MarkerRecognition
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class waypoint_identifier:

    def __init__(self):
        self.image_pub = rospy.Publisher("marker/image_feed", Image, queue_size=1)
        self.mask_pub = rospy.Publisher("marker/mask_feed", Image, queue_size=1)
        self.eroded_pub = rospy.Publisher("marker/eroded_feed", Image, queue_size=1)
        self.dilated_pub = rospy.Publisher("marker/dilated_feed", Image, queue_size=1)
        self.recognition_pub = rospy.Publisher("marker/recognition", MarkerRecognition, queue_size=1)

        self.bridge = CvBridge()
        self.boundsHSV = rospy.Subscriber("hsv_bounds", HSVbounds, self.setSpaceCallback, queue_size=1)
        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.callback, queue_size=1)

        self.currentPos = [0, 0, 0, 0]  # current x,y,w,h
        self.previousPos = [0, 0, 0, 0]  # previous x,y,w,h
        self.lower_bound = np.array([0.0, 0.0, 0.0])
        self.upper_bound = np.array([0.0, 0.0, 0.0])
        self.previousArea = 0
        self.currentArea = 0
        self.areaTimerON = False
        self.timerStart = time.time()
        self.camFOV = 80  # camera field of view (degrees)

    def setSpaceCallback(self, data):
        self.lower_bound = np.array(data.lower)
        self.upper_bound = np.array(data.upper)

    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        recognition = MarkerRecognition()

        # Noise filtering (median or Gaussian filter)
        # blur_img = cv2.medianBlur(frame,5)
        blur_img = cv2.GaussianBlur(frame, (5, 5), 5)

        hsv_img = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, self.lower_bound, self.upper_bound)

        #Get rid of background noise using erosion and fill in the holes using dilation
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        eroded = cv2.erode(mask, element, iterations=2)
        dilated = cv2.dilate(eroded, element, iterations=6)

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
        if abs(self.currentPos[0]-self.previousPos[0]) < 50 and abs(self.currentPos[1]-self.previousPos[1]) < 50 and abs(self.currentArea-self.previousArea) < (self.currentArea*0.3) and (not self.areaTimeout):
            stable = True
            rectColor = [0, 0, 255]
        else:
            stable = False
            self.timerStart = time.time()

        # If detection is consistent for 3 seconds, confirm that the marker is detected
        if stable and abs(time.time()-self.timerStart) > 3:
            markerLocked = True
            rectColor = [255, 0, 0]
        else:
            markerLocked = False

        # Draws rectangle around detection (red means preliminary detection, blue means detection confirmed)
        if stable:
            cv2.rectangle(frame, (self.currentPos[0], self.currentPos[1]), (self.currentPos[0]+self.currentPos[3], self.currentPos[1]+self.currentPos[3]), rectColor, 3)

        self.previousPos = self.currentPos
        self.previousArea = self.currentArea

        if markerLocked:
            #Output marker angular position from the center of the feed (in rad)
            #Center is 0 rad, left is +ve, right is -ve
            offset = 2*math.pi*((frame.shape[1]/2.0-(self.currentPos[0]+self.currentPos[2]/2.0))/(frame.shape[1]/2.0)*(self.camFOV/2))/360.0
            stopCommand = self.currentArea > 8000
            print offset, stopCommand
        else:
            stopCommand = False
            offset = 0.0

        recognition.locked = markerLocked
        recognition.inRange = stopCommand
        recognition.orientation = offset

        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
            self.eroded_pub.publish(self.bridge.cv2_to_imgmsg(eroded, "mono8"))
            self.dilated_pub.publish(self.bridge.cv2_to_imgmsg(dilated, "mono8"))
            self.recognition_pub.publish(recognition)
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