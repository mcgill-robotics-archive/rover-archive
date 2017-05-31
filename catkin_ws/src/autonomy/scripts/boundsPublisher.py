#!/usr/bin/env python

import rospy
from autonomy.msg import HSVbounds


if __name__ == '__main__':
    rospy.init_node('marker_bounds_publisher', anonymous=False)
    hsv_pub = rospy.Publisher("hsv_bounds", HSVbounds, queue_size=10)

    trackedHSV = [111.43, 255.0, 126.0]

    hsvSpace = HSVbounds()
    hsvSpace.lower = [trackedHSV[0]-10, trackedHSV[1]-50, trackedHSV[2]-70]
    hsvSpace.upper = [trackedHSV[0]+10, trackedHSV[1]+50, trackedHSV[2]+70]

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        hsv_pub.publish(hsvSpace)
        rate.sleep()
