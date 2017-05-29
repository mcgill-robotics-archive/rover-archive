#!/usr/bin/env python
import rospy
from ahrs.msg import AhrsStdMsg
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped


class NavSatMsgManager:
    def __init__(self):
        rospy.init_node('ahrs_manager')
        rospy.loginfo("Starting AHRS manager node...")
        
        self.frame_id = rospy.get_param("frame_id", "base_link")

        self.ahrs_navsat_pub = rospy.Publisher('~gps_fix', NavSatFix,
                                          queue_size=10)

        self.ahrs_pose_pub = rospy.Publisher("~imu_data",
                                             PoseWithCovarianceStamped,
                                             queue_size=1)

        rospy.Subscriber("/ahrs/ahrs_status", AhrsStdMsg, self.ahrs_callback)

    def pub_ahrs_pose(self, orientation):
        ahrs_pose = PoseWithCovarianceStamped()
        ahrs_pose.pose.pose.orientation = orientation
        ahrs_pose.pose.covariance = [0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 1e-6, 0, 0,
                                     0, 0, 0, 0, 1e-6, 0,
                                     0, 0, 0, 0, 0, 1e-6]
        self.ahrs_pose_pub.publish(ahrs_pose)

    def pub_ahrs_navsat(self, data):
        navsat = NavSatFix()
        
        navsat.header.stamp = rospy.Time.now()
        navsat.header.frame_id = "base_link"
        
        navsat.latitude = data.gps.latitude
        navsat.longitude = data.gps.longitude
        navsat.altitude = data.gps.altitude

        navsat.position_covariance[0]= (data.gps.horiAccuracy / 1000.0) ** 2
        navsat.position_covariance[4]= (data.gps.horiAccuracy / 1000.0) ** 2
        navsat.position_covariance[8]= (data.gps.vertAccuracy / 1000.0) ** 2
        navsat.position_covariance_type = \
                navsat.COVARIANCE_TYPE_APPROXIMATED
        
        if data.gps.FIX_3D:
            navsat.status.status = navsat.status.STATUS_GBAS_FIX
        else:
            navsat.status.status = navsat.status.STATUS_NO_FIX
        
        navsat.status.service = navsat.status.SERVICE_GPS
        
        self.ahrs_navsat_pub.publish(navsat)

    def ahrs_callback(self, data):
        self.header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        self.pub_ahrs_pose(data.pose.pose.orientation)
        
        if data.gps.validUTC:
            self.pub_ahrs_navsat(data)

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.imu_received:
                pass
            rate.sleep()


if __name__ == '__main__':
    nav_sat_manager = NavSatMsgManager()
    nav_sat_manager.run()
