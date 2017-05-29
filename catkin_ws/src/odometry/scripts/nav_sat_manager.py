#!/usr/bin/env python
import rospy
from ahrs.msg import AhrsStdMsg
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped


class NavSatMsgManager:
    def __init__(self):
        rospy.init_node('navsat_manager')

        self.imu_received = False

        self.navsat_pub = rospy.Publisher('ahrs_raw_gps', NavSatFix,
                                          queue_size=10)

        self.ahrs_pose_pub = rospy.Publisher("~imu_pose",
                                             PoseWithCovarianceStamped,
                                             queue_size=1)

        rospy.Subscriber("/ahrs/ahrs_status", AhrsStdMsg, self.ahrs_callback)

    def pub_ahrs_pose(self, orientation):
        ahrs_pose = PoseWithCovarianceStamped()
        ahrs_pose.header.stamp = rospy.Time.now()
        ahrs_pose.header.frame_id = "base_link"
        ahrs_pose.pose.pose.orientation = orientation
        ahrs_pose.pose.covariance = [0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 1e-6, 0, 0,
                                     0, 0, 0, 0, 1e-6, 0,
                                     0, 0, 0, 0, 0, 1e-6]
        self.ahrs_pose_pub.publish(ahrs_pose)

    def ahrs_callback(self, data):
        self.pub_ahrs_pose(data.pose.pose.orientation)
        # Get the Pose data from the AhrsStdMsg (i.e. the IMU data).
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
                pass
            rate.sleep()


if __name__ == '__main__':
    nav_sat_manager = NavSatMsgManager()
    nav_sat_manager.run()
