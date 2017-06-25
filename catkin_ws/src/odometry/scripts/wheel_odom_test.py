#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped


class WheelOdomTest(object):
    def __init__(self):
        self.last_time = rospy.Time.now()
        self.current_velocity = None
        self.distance_travelled = 0.0
        rospy.Subscriber("/wheel_odom/wheel_velocity",
                         TwistWithCovarianceStamped,
                         self.callback)

    def callback(self, data):
        self.current_velocity = data.twist.twist.linear.x
        current_time = rospy.Time.now()
        self.distance_travelled += self.current_velocity * (
                current_time - self.last_time).to_sec()
        self.last_time = current_time
        rospy.loginfo("%f meters travelled" % self.distance_travelled)


if __name__ == "__main__":
    rospy.init_node("wheel_odom_measurement")
    WheelOdomTest()
    rospy.spin()
