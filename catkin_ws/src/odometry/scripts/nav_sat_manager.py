#!/usr/bin/env python
import rospy
from ahrs.msg import AhrsStdMsg
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu


class NavSatMsgManager:
    def __init__(self):
        rospy.init_node('navsat_manager')

        self.imu_received = False
        self.current_imu = Imu()

        self.navsat_pub = rospy.Publisher('ahrs_raw_gps', NavSatFix,
                                          queue_size=10)
        self.imu_pub = rospy.Publisher('ahrs_raw_imu', Imu,
                                       queue_size=10)

        rospy.Subscriber("/ahrs/ahrs_status", AhrsStdMsg, self.ahrs_callback)

    def ahrs_callback(self, data):
        # Get the Pose data from the AhrsStdMsg (i.e. the IMU data).
        ahrs_imu_data = Imu()
        ahrs_imu_data.orientation = data.pose.pose.orientation
        ahrs_imu_data.orientation_covariance = [
            10e-5, 0, 0,
            0, 10e-5, 0,
            0, 0, 10e-5
        ]

        ahrs_imu_data.header.frame_id = "ahrs_imu_frame"
        ahrs_imu_data.header.stamp = rospy.get_rostime()
        self.current_imu = ahrs_imu_data
        self.imu_received = True

        # Create the NavSatFix message, then populate the gps data
        # navsatmsg = NavSatFix()
        # navsatmsg.latitude = data.gps.latitude
        # navsatmsg.longitude = data.gps.longitude
        # navsatmsg.altitude = data.gps.altitude

        # Publish the NavSatFix message
        # if data.gps.validUTC:
        #    self.navsat_pub.publish(navsatmsg)

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.imu_received:
                self.imu_pub.publish(self.current_imu)
                rate.sleep()


if __name__ == '__main__':
    navSatManager = NavSatMsgManager()
    navSatManager.run()
