#!/usr/bin/env python

"""ROS node that takes wheel encoder values and broadcasts wheel odometry."""

import rospy
import message_filters
from rover_common.msg import DriveEncoderStamped
from geometry_msgs.msg import TwistStamped, Twist


class WheelOdom(object):
    """Compute wheel odometry based on encoder readings and physical ratios."""

    AXEL_DIAMETER = 1.405  # Meters.

    def differential_velocity_estimation(self, lwv, rwv):
        """Calculate velocity based on wheel velocities."""
        diff_velocity = Twist()

        # The robot does not move laterally nor does move vertically.
        diff_velocity.linear.y = 0.0
        diff_velocity.linear.z = 0.0

        # The wheel odomtery can only measure the yaw velocity.
        diff_velocity.angular.x = 0.0
        diff_velocity.angular.y = 0.0

        # Helper local variable for notation purposes.
        l = WheelOdom.AXEL_DIAMETER

        # Cases:
        # 1 - The wheels are travelling in same direction at same speed.
        # 2 - Left wheel is moving at the same speed but opposite dir to right.
        # 3 - We are rotating about the right wheel.
        # 4 - We are rotating about the left wheel.
        # 5 - All other cases.
        if lwv == rwv:
            # The x dir velocity is the same as the left and right wheels.
            diff_velocity.linear.x = rwv
            # The angular velocity is 0.
            diff_velocity.angular.z = 0.0

        elif lwv == -rwv:
            # There is no linear movement, only yaw.
            diff_velocity.linear.x = 0.0
            # The yaw is (rwv - lwv) / Diameter of the robot
            diff_velocity.angular.z = 2 * rwv / l

        elif rwv == 0.0 or lwv == 0.0:
            # We are rotating about the left or right wheel.
            turn_radius = l / 2.0
            yaw_angular_velocity = (rwv - lwv) / l

            diff_velocity.linear.x = yaw_angular_velocity * turn_radius
            diff_velocity.angular.z = yaw_angular_velocity

        else:
            # All other sittuations we just do the full calculation.
            turn_radius = (l / 2.0) * (lwv + rwv) / (rwv - lwv)
            yaw_angular_velocity = (rwv - lwv) / l

            diff_velocity.linear = turn_radius * yaw_angular_velocity
            diff_velocity.angular = yaw_angular_velocity

        return diff_velocity

    def raw_encoder_to_twist(self, left_wheel_enc, right_wheel_enc):
        """Calculate a twist message based on the given raw encoder values."""
        # Convert the raw data to physical velocities.
        left_wheel_speed = self.tacho_to_mps(left_wheel_enc.taco_velocity)
        right_wheel_speed = self.tacho_to_mps(right_wheel_enc.tachos_velocity)

        left_wheel_dir = left_wheel_enc.direction
        right_wheel_dir = right_wheel_enc.direction

        # Linear velocity of the wheels.
        vel_lw = left_wheel_speed * left_wheel_dir
        vel_rw = right_wheel_speed * right_wheel_dir

        return self.differential_velocity_estimation(vel_lw, vel_rw)

    def odometry_callback(self, enc_stamped_lw, enc_stamped_rw):
        """ROS odometry subscriber callback."""
        encoder_lw = enc_stamped_lw.drive_encoder
        encoder_rw = enc_stamped_rw.driver_encoder

        # Compute the estimated rover velocity based on encoder readings.
        rover_velocity = self.raw_encoder_to_twist(encoder_lw, encoder_rw)

        rover_velocity_stamped = TwistStamped()
        rover_velocity_stamped.twist = rover_velocity
        rover_velocity_stamped.header.stamp = rospy.get_rostime()

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

        # TODO: Set up publishers. Test.
        rospy.spin()

    def tacho_to_mps(self, tachos_ps):
        """Take the tachos per second and converts it to meters per second."""
        tacho_disk_rotation_ratio = 2000  # Design parameter.
        disk_shaft_rotation_ratio = 21  # Disk to shaft.

        # Based on circ of wheel with average wheel radius.
        # Max wheel dia: 10.5 in. Min wheel diam: 10.25 in
        # Average : 10.375 in
        shaft_to_meters_rotation_ratio = 0.827888

        disk_rotations_ps = tachos_ps / tacho_disk_rotation_ratio
        shaft_rotations_ps = disk_rotations_ps / disk_shaft_rotation_ratio
        meters_ps = shaft_rotations_ps / shaft_to_meters_rotation_ratio

        return meters_ps


if __name__ == "__main__":
    WheelOdom()
