#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage

img_left = None
img_right = None
new_left = False
new_right = False

def combine(left_frame, right_frame):

    left_cv_image = cv2.imdecode(np.fromstring(left_frame.data, np.uint8), cv2.IMREAD_COLOR)
    right_cv_image = cv2.imdecode(np.fromstring(right_frame.data, np.uint8), cv2.IMREAD_COLOR)

    combined_image = np.zeros((left_cv_image.shape[0]+right_cv_image.shape[0], left_cv_image.shape[1], 3), np.uint8)

    combined_image[0:left_cv_image.shape[0], 0:left_cv_image.shape[1]] = left_cv_image
    combined_image[left_cv_image.shape[0]:combined_image.shape[0], 0:combined_image.shape[1]] = right_cv_image

    comb_img = CompressedImage()
    comb_img.data = np.array(cv2.imencode(".jpg", combined_image)[1]).tostring()
    comb_img.header = left_frame.header
    comb_img.format = "jpeg"

    return comb_img

def left_cb(message):
    global img_left, new_left
    img_left = message
    new_left = True

def right_cb(message):
    global img_right, new_right
    img_right = message
    new_right = True


if __name__ == '__main__':

    rospy.init_node('navcam_combiner', anonymous=False)
    comp_pub = rospy.Publisher("camera/combined", CompressedImage, queue_size=10)
    left_nav_cam_topic = rospy.get_param("~nav_cam_left", "cam/camera/compressed")
    right_nav_cam_topic = rospy.get_param("~nav_cam_right", "cam/camera/compressed")

    rospy.Subscriber(left_nav_cam_topic, CompressedImage, left_cb)
    rospy.Subscriber(right_nav_cam_topic, CompressedImage, right_cb)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if (new_right and new_left):
            combined_img = combine(img_left, img_right)
            comp_pub.publish(combined_img)
            new_left = False
            new_right = False

        rate.sleep()
