#!/usr/bin/env python

"""ROS node that takes wheel encoder values and broadcasts wheel odometry."""

import rospy
from rover_common.msg import DriveEncoderStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import TwistWithCovarianceStamped


class WheelOdom(object):
    """Compute wheel odometry based on encoder readings and physical ratios."""

    AXEL_DIAMETER = 1.405  # Meters.

    ODOM_ERR_FACTOR = 17.24

    COVARIANCE_MATRIX = [
        1e-6, 0, 0, 0, 0, 0,  # We are estimating the forward velocity.
        0, 1e-9, 0, 0, 0, 0,  # No instantaneous Y velocity b/c diff drive.
        0, 0, 1e-9, 0, 0, 0,  # We cannot have a Z velocity.
        0, 0, 0, 0, 0, 0,     # We can not estimate anything about roll.
        0, 0, 0, 0, 0, 0,     # Nor about pitch.
        0, 0, 0, 0, 0, 0   # We are estimating the yaw of the rover.
    ]

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

            diff_velocity.linear.x = turn_radius * yaw_angular_velocity
            diff_velocity.angular.z = yaw_angular_velocity

        return diff_velocity

    def raw_encoder_to_twist(self, left_wheel_enc, right_wheel_enc):
        """Calculate a twist message based on the given raw encoder values."""
        # Convert the raw data to physical velocities.
        left_wheel_speed = self.tacho_to_mps(left_wheel_enc.taco_velocity)
        right_wheel_speed = self.tacho_to_mps(right_wheel_enc.taco_velocity)

        left_wheel_dir = left_wheel_enc.direction * -1
        right_wheel_dir = right_wheel_enc.direction

        # Linear velocity of the wheels.
        vel_l = left_wheel_speed * left_wheel_dir * WheelOdom.ODOM_ERR_FACTOR
        vel_r = right_wheel_speed * right_wheel_dir * WheelOdom.ODOM_ERR_FACTOR
        return self.differential_velocity_estimation(vel_l, vel_r)

    def run(self):
        """Run the publisher in a loop."""
        rate = rospy.Rate(50)  # 50 Hz, the encoders are running at 30 Hz
        while not rospy.is_shutdown():
            msgs_rec = self.messages_received
            if msgs_rec["left"] and msgs_rec["right"]:
                self.publish_odometry()
            else:
                rospy.logdebug("No messages yet received.")

            rate.sleep()

    def publish_odometry(self):
        """Publish odometry."""
        encoder_lw = self.encoder_values['left']
        encoder_rw = self.encoder_values['right']

        # Compute the estimated rover velocity based on encoder readings.
        rover_velocity = self.raw_encoder_to_twist(encoder_lw, encoder_rw)

        vel_with_cov = TwistWithCovariance()
        vel_with_cov.twist = rover_velocity
        vel_with_cov.covariance = WheelOdom.COVARIANCE_MATRIX

        rover_velocity_stamped = TwistWithCovarianceStamped()
        rover_velocity_stamped.twist = vel_with_cov
        rover_velocity_stamped.header.stamp = rospy.get_rostime()

        self.velocity_publisher.publish(rover_velocity_stamped)

    def encoder_callback(self, encoder_stamped, args):
        self.encoder_values[args] = encoder_stamped.drive_encoder
        self.messages_received[args] = True

    def __init__(self):
        """Initialize the wheel odometry node and subscriber callbacks."""
        rospy.init_node('odometry', log_level=rospy.DEBUG)
        rospy.logdebug('Initialized nodes')

        self.encoder_values = {}
        self.messages_received = {"left": False, "right": False}

        # Topic names.
        lwt = 'encoder_left'
        rwt = 'encoder_right'

        rospy.Subscriber(lwt, DriveEncoderStamped,
                         self.encoder_callback, ("left"))
        rospy.Subscriber(rwt, DriveEncoderStamped,
                         self.encoder_callback, ("right"))

        self.velocity_publisher = rospy.Publisher("~wheel_velocity",
                                                  TwistWithCovarianceStamped,
                                                  queue_size=5)

    def tacho_to_mps(self, tachos_ps):
        """Take the tachos per second and converts it to meters per second."""
        tacho_disk_rotation_ratio = 2000  # Design parameter.
        disk_shaft_rotation_ratio = 21    # Disk to shaft.

        # Based on circ of wheel with average wheel radius.
        # Max wheel dia: 10.5 in. Min wheel diam: 10.25 in
        # Average : 10.375 in
        shaft_to_meters_rotation_ratio = 0.827888

        disk_rotations_ps = float(tachos_ps) / tacho_disk_rotation_ratio
        shaft_rotations_ps = disk_rotations_ps / disk_shaft_rotation_ratio
        meters_ps = shaft_rotations_ps / shaft_to_meters_rotation_ratio

        return meters_ps


if __name__ == "__main__":
    x = WheelOdom()
    x.run()
