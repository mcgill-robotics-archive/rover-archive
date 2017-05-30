#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage


def combine(left_frame, right_frame):

    comb_img = CompressedImage()

    left_cv_image = cv2.imdecode(np.fromstring(left_frame.data, np.uint8), cv2.IMREAD_COLOR)
    right_cv_image = cv2.imdecode(np.fromstring(right_frame.data, np.uint8), cv2.IMREAD_COLOR)

    combined_image = np.zeros((left_cv_image.shape[0], left_cv_image.shape[1]+right_cv_image.shape[1], 3), np.uint8)

    combined_image[0:left_cv_image.shape[0], 0:left_cv_image.shape[1]] = left_cv_image
    combined_image[0:right_cv_image.shape[0], left_cv_image.shape[1]:combined_image.shape[1]] = right_cv_image

    comb_img.data = np.array(cv2.imencode(".jpg", combined_image)[1]).tostring()
    comb_img.header = left_frame.header
    comb_img.format = "jpeg"

    return comb_img


if __name__ == '__main__':

    rospy.init_node('navcam_combiner', anonymous=False)
    comp_pub = rospy.Publisher("camera/combined", CompressedImage, queue_size=10)
    left_nav_cam_topic = rospy.get_param("~nav_cam_left", "hazcamera1/camera/compressed")
    right_nav_cam_topic = rospy.get_param("~nav_cam_right", "hazcamera2/camera/compressed")

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        img_left = rospy.wait_for_message(left_nav_cam_topic, CompressedImage)
        img_right = rospy.wait_for_message(right_nav_cam_topic, CompressedImage)

        combined_img = combine(img_left, img_right)
        comp_pub.publish(combined_img)

        rate.sleep()
