#!/usr/bin/env python

import rospy
from ahrs.msg import AhrsStdMsg
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3, Quaternion
import tf
from numpy import pi

class NavSatMsgManager:

    ACCEL_VARIANCE = (0.25 / 1000) ** 2 * 250
    GYRO_VARIANCE = (0.05 / 1000 * pi / 180.0) ** 2 * 240

    def __init__(self):
        rospy.init_node('ahrs_manager')
        rospy.loginfo("Starting AHRS manager node...")
        self.pose_discarded = 0.0
        self.POSE_DISCARD_COUNT = 200.0
        self.ahrs_local_frame = "odom"
        self.ahrs_global_frame = rospy.get_param("ahrs_global_frame",
                                                 "base_link")

        self.ahrs_navsat_pub = rospy.Publisher('~gps', NavSatFix,
                                          queue_size=1)

        self.ahrs_imu_pub = rospy.Publisher('~imu', Imu,
                                          queue_size=1)

        self.ahrs_pose_pub = rospy.Publisher("~pose",
                                             PoseWithCovarianceStamped,
                                             queue_size=1)

        rospy.Subscriber("/ahrs/ahrs_status", AhrsStdMsg, self.ahrs_callback)

    def pub_ahrs_imu(self, data):
        """Publish IMU message for NavSat node."""
        if data.gps.fix_type == data.gps.FIX_TYPE_FIX_3D:
            rospy.loginfo_throttle(60000, "GPS 3D Fixed!")

            if self.pose_discarded < self.POSE_DISCARD_COUNT:
                self.pose_discarded += 1
                rospy.loginfo_throttle(2, 'Discarding first' +
                        ' few messages, {0:.2%} Complete...'.format(
                                self.pose_discarded / self.POSE_DISCARD_COUNT))
            else:
                rospy.logdebug_throttle(1, 'Current Heading:' +
                '{0:.2f} degree' .format(data.gps.heading))

                heading = data.gps.heading * pi / 180.0
                heading_accuracy = data.gps.heading_accuracy * pi / 180.0

                q = data.pose.pose.orientation
                euler = tf.transformations.euler_from_quaternion(
                        [q.x, q.y, q.z, q.w])

                q = tf.transformations.quaternion_from_euler(
                        euler[0], euler[1], heading)

                msg = Imu()
                msg.header = self.header_global

                msg.orientation.x = q[0]
                msg.orientation.y = q[1]
                msg.orientation.z = q[2]
                msg.orientation.w = q[3]
                msg.orientation_covariance[0] = data.attitude_accuracy ** 2
                msg.orientation_covariance[4] = data.attitude_accuracy ** 2
                msg.orientation_covariance[8] = heading_accuracy ** 2

                msg.angular_velocity.x = data.gyroscopes.x
                msg.angular_velocity.y = data.gyroscopes.y
                msg.angular_velocity.z = data.gyroscopes.z
                msg.angular_velocity_covariance[0] = self.GYRO_VARIANCE
                msg.angular_velocity_covariance[4] = self.GYRO_VARIANCE
                msg.angular_velocity_covariance[8] = self.GYRO_VARIANCE

                msg.linear_acceleration.x = data.accelerometers.x
                msg.linear_acceleration.y = data.accelerometers.y
                msg.linear_acceleration.z = data.accelerometers.z
                msg.linear_acceleration_covariance[0] = self.ACCEL_VARIANCE
                msg.linear_acceleration_covariance[4] = self.ACCEL_VARIANCE
                msg.linear_acceleration_covariance[8] = self.ACCEL_VARIANCE

                self.ahrs_imu_pub.publish(msg)


    def pub_ahrs_pose(self, data):
        """Publish the AHRS pose for local state estimation."""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = self.header_local
        pose_msg.pose.pose.position = data.pose.pose.position
        quat = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quat)
        euler_fixed = (euler[0], -1 * euler[1], -1 * euler[2])

        quat_fixed = tf.transformations.quaternion_from_euler(*euler_fixed)

        pose_msg.pose.pose.orientation = Quaternion(*list(quat_fixed))
        
        pose_msg.pose.covariance[0] = data.position_accuracy ** 2
        pose_msg.pose.covariance[7] = data.position_accuracy ** 2
        pose_msg.pose.covariance[14] = data.position_accuracy ** 2
        pose_msg.pose.covariance[21] = data.attitude_accuracy ** 2
        pose_msg.pose.covariance[28] = data.attitude_accuracy ** 2
        pose_msg.pose.covariance[35] = data.attitude_accuracy ** 2

        self.ahrs_pose_pub.publish(pose_msg)

    def pub_ahrs_navsat(self, data):
        """Publish NavSatFix message for NavSat node."""
        navsat = NavSatFix()

        navsat.header = self.header_global

        navsat.latitude = data.gps.latitude
        navsat.longitude = data.gps.longitude
        navsat.altitude = data.gps.altitude

        navsat.position_covariance[0]= data.gps.horizontal_accuracy ** 2
        navsat.position_covariance[4]= data.gps.horizontal_accuracy ** 2
        navsat.position_covariance[8]= data.gps.vertical_accuracy ** 2
        navsat.position_covariance_type = navsat.COVARIANCE_TYPE_APPROXIMATED

        if data.gps.fix_type == data.gps.FIX_TYPE_FIX_3D:
            navsat.status.status = navsat.status.STATUS_FIX
        else:
            navsat.status.status = navsat.status.STATUS_NO_FIX

        navsat.status.service = navsat.status.SERVICE_GPS

        self.ahrs_navsat_pub.publish(navsat)

    def ahrs_callback(self, data):
        self.header_local = Header(stamp=rospy.Time.now(),
                                   frame_id=self.ahrs_local_frame)
        self.header_global = Header(stamp=rospy.Time.now(),
                                    frame_id=self.ahrs_global_frame)
        self.pub_ahrs_pose(data)
        self.pub_ahrs_imu(data)
        if data.gps.validUTC:
            self.pub_ahrs_navsat(data)

if __name__ == '__main__':
    nav_sat_manager = NavSatMsgManager()
    rospy.spin()
