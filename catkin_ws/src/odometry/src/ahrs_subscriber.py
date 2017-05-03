#!/usr/bin/env python
import rospy
import std_msgs
from ahrs.msg import AhrsStdMsg
from sensor_msgs.msg import NavSatFix
class NavSatMsgGPS:
    def __init__(self):
        self.publisher = rospy.Publisher('ahrs_raw_gps', NavSatFix, queue_size = 10)
    

def callback(data):

    #create the NavSatFix message, then populate the gps data
    navsatmsg = NavSatFix()
    navsatmsg.latitude = data.gps.latitude
    navsatmsg.longitude = data.gps.longitude
    navsatmsg.altitude = data.gps.altitude

    #publish the NavSatFix message
    navSatMsgGPS = NavSatMsgGPS()
    navSatMsgGPS.publisher.publish(navsatmsg)
    

def listener():

    rospy.init_node('ahrs_listener', anonymous=True)
    rospy.Subscriber("ahrs_status", AhrsStdMsg, callback)
    

    
    
        
       

if __name__ == '__main__':
    
    while not rospy.is_shutdown():
        listener()
        rospy.spin()
        
