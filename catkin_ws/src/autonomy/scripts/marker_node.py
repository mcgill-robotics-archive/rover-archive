#!/usr/bin/env python

"""ROS node for identifying waypoint markers at URC."""

import sys
import time
import math
import rospy
import cv2
import numpy as np
from autonomy.msg import HSVbounds, MarkerRecognition
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class WaypointIdentifier:
    """Encompass all functions and ROS interfacing for waypoint recog."""
    def __init__(self):
        self.image_pub = rospy.Publisher("marker/image_feed", Image,
                                         queue_size=1)

        self.mask_pub = rospy.Publisher("marker/mask_feed", Image,
                                        queue_size=1)

        self.eroded_pub = rospy.Publisher("marker/eroded_feed", Image,
                                          queue_size=1)

        self.dilated_pub = rospy.Publisher("marker/dilated_feed", Image,
                                           queue_size=1)

        self.recognition_pub = rospy.Publisher("marker/recognition",
                                               MarkerRecognition,
                                               queue_size=1)

        self.bridge = CvBridge()
        self.hsv_bounds = rospy.Subscriber("hsv_bounds", HSVbounds,
                                           self.hsv_bound_callback, queue_size=1)

        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image,
                                          self.recognition_callback,
                                          queue_size=1)

        self.current_pos = [0, 0, 0, 0]  # x, y, w, h
        self.previous_pos = [0, 0, 0, 0]  # x, y, w, h
        self.lower_bound = np.array([0.0, 0.0, 0.0])
        self.upper_bound = np.array([0.0, 0.0, 0.0])
        self.previous_area = 0
        self.current_area = 0
        self.area_timer = None
        self.area_timeout = None
        self.area_timer_on = False
        self.timer_start = time.time()
        self.camera_fov = 80  # camera field of view (degrees)

    def hsv_bound_callback(self, data):
        """Set lower and upper bound for HSVs to look for."""
        self.lower_bound = np.array(data.lower)
        self.upper_bound = np.array(data.upper)

    def recognition_callback(self, data):
        """Main callback for our image recognition procedure."""
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as err:
            print(err)

        recognition = MarkerRecognition()

        # Noise filtering (median or Gaussian filter)
        blur_img = cv2.GaussianBlur(frame, (5, 5), 5)

        hsv_img = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, self.lower_bound, self.upper_bound)

        # Get rid of background noise using erosion and fill in using dilation
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        eroded = cv2.erode(mask, element, iterations=3)
        dilated = cv2.dilate(eroded, element, iterations=6)

        # Identify the right feature on the dilated image
        img3, contours, hierarchy = cv2.findContours(dilated, cv2.RETR_LIST,
                                                     cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            self.current_area = cv2.contourArea(contour)
            self.current_pos = cv2.boundingRect(contour)

        # Look for false positives
        if self.previous_area == self.current_area:
            if not self.area_timer_on:
                self.area_timer = time.time()
                self.area_timer_on = True
            else:
                self.area_timeout = (time.time() - self.area_timer) > 1
        else:
            self.area_timeout = False
            self.area_timer_on = False

        # Verify if our detection is stable & consistent
        if (abs(self.current_pos[0]-self.previous_pos[0]) < 50
                and abs(self.current_pos[1]-self.previous_pos[1]) < 50
                and abs(self.current_area-self.previous_area) <
                (self.current_area*0.3)
                and (not self.area_timeout)):
            stable = True
            rect_colour = [0, 0, 255]
        else:
            stable = False
            self.timer_start = time.time()

        # If detection is consistent for 3 seconds, confirm that the marker is detected
        if stable and abs(time.time()-self.timer_start) > 3:
            marker_locked = True
            rect_colour = [255, 0, 0]
        else:
            marker_locked = False

        # Draws rectangle around detection (red-preliminary, blue-confirmed)
        if stable:
            cv2.rectangle(frame, (self.current_pos[0], self.current_pos[1]),
                          (self.current_pos[0]+self.current_pos[3],
                           self.current_pos[1]+self.current_pos[3]),
                          rect_colour, 3)

        self.previous_pos = self.current_pos
        self.previous_area = self.current_area

        if marker_locked:
            #Output marker angular position from the center of the feed (in rad)
            #Center is 0 rad, left is +ve, right is -ve
            offset = 2*math.pi*(
                (frame.shape[1]/2.0 - (self.current_pos[0] +
                                       self.current_pos[2]/2.0)) /
                (frame.shape[1]/2.0)*(self.camera_fov/2))/360.0
            stop_command = self.current_area > 8000
            print offset, stop_command
        else:
            stop_command = False
            offset = 0.0

        recognition.locked = marker_locked
        recognition.inRange = stop_command
        recognition.orientation = offset

        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
            self.eroded_pub.publish(self.bridge.cv2_to_imgmsg(eroded, "mono8"))
            self.dilated_pub.publish(self.bridge.cv2_to_imgmsg(dilated, "mono8"))
            self.recognition_pub.publish(recognition)
        except CvBridgeError as err:
            print(err)


def main(_):
    """Handle node and identifier initialization."""
    rospy.init_node('marker_navigator', anonymous=True)
    WaypointIdentifier()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
