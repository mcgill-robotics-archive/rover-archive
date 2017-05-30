#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


def acquire(cap, device_id):

    bridge = CvBridge()
    comp_img = CompressedImage()

    _, frame = cap.read()

    try:
        ros_img = bridge.cv2_to_imgmsg(frame, "bgr8")
    except CvBridgeError as err:
        print(err)

    comp_img.data = np.array(cv2.imencode(".jpg", frame)[1]).tostring()
    comp_img.header = ros_img.header
    comp_img.format = "jpeg"

    return comp_img


if __name__ == '__main__':

    rospy.init_node('navcam_acquisitioner', anonymous=False)
    # img_pub = rospy.Publisher("camera", Image, queue_size=10)
    comp_pub = rospy.Publisher("camera/compressed", CompressedImage, queue_size=10)
    device_name = rospy.get_param("~device_id", "/dev/hazcam0")
    device_id = int(device_name[-1])
    capture = cv2.VideoCapture(device_id)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        compressed = acquire(capture, device_id)
        # img_pub.publish(image)
        comp_pub.publish(compressed)

        rate.sleep()
