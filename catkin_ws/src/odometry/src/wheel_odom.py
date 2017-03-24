#!/usr/bin/env python

"""ROS node that takes wheel encoder values and broadcasts wheel odometry."""

import rospy
import message_filters
from rover_common.msg import DriveEncoderStamped


class WheelOdom(object):
    """Compute wheel odometry based on encoder readings and physical ratios."""

    def odometry_callback(self, left_wheel_encoder, right_wheel_encoder):
        """ROS odometry subscriber callback."""
        pass

    def __init__(self):
        """Initialize the wheel odometry node and subscriber callbacks."""
        rospy.init_node('odometry')

        # Topic names.
        lwt = 'left_wheel_encoder'
        rwt = 'right_wheel_encoder'

        # Define subscribers.
        l_enc_sub = message_filters.Subscriber(lwt, DriveEncoderStamped)
        r_enc_sub = message_filters.Subscriber(rwt, DriveEncoderStamped)

        # Synchronize subscribers.
        ts = message_filters.TimeSynchronizer([l_enc_sub, r_enc_sub], 5)

        # Register the odometry callback for synchronized subscribers.
        ts.registerCallback(self.odometry_callback)
        rospy.spin()

    def tachometer_to_meters(self, tacho_count_left, tacho_count_right):
        """Take the tacho count and converts it to meters."""
        tacho_to_disk_rotation_ratio = 2000  # Design parameter.
        # Based on circ of wheel with average hweel radius.
        # Max wheel dia: 10.5 in. Min wheel diam: 10.25 in
        # Average : 10.375 cm
        disk_to_shaft_rotation_ratio = 21  # Disk to shaft.
        shaft_to_meters_rotation_ratio = 0.827888


if __name__ == "__main__":
    WheelOdom()
