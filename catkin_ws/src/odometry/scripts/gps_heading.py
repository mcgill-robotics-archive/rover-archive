#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from ahrs.msg import AhrsStdMsg, Ig500nGPS 
from std_msgs.msg import Header
from numpy import pi
import tf


# Global Constants
DISCARD_COUNT = 200


# Global Variables
heading_count = 0.0


def ahrs_callback(msg):
    '''Callback function for receiving ARHS message.

    Arg:
        msg: ROS msg object for the callback.
    '''
    global heading_count
    # Create new message and header
    
    if msg.gps.FIX_3D :
        rospy.loginfo_throttle(60, "GPS 3D Fixed!")
        if heading_count < DISCARD_COUNT :
            heading_count += 1
            rospy.loginfo_throttle(1, 
                    'Discarding first few messages: {0:.2%} Complete'.format(
                            heading_count / DISCARD_COUNT))
        else :
            rospy.loginfo_throttle(1, "Current Heading: {0:.2f} degree".format(
                    msg.gpsHeading.data / 100000.0))
            robot_pose = PoseStamped(header=Header(frame_id=frame_id, 
                    stamp=rospy.Time.now()))
            heading = msg.gpsHeading.data * pi / 180 / 100000
            q = tf.transformations.quaternion_from_euler(0, 0, heading)
            robot_pose.pose.orientation.x = q[0]
            robot_pose.pose.orientation.y = q[1]
            robot_pose.pose.orientation.z = q[2]
            robot_pose.pose.orientation.w = q[3]
            pub.publish(robot_pose)

if __name__ == '__main__':
    # Init the ROS node
    rospy.init_node('gps_heading')
    rospy.loginfo("GPS Heading node started")
    frame_id = rospy.get_param("~frame_id", "base_link")
    sub = rospy.Subscriber('/ahrs/ahrs_status', AhrsStdMsg,
            ahrs_callback, queue_size=1)
    pub = rospy.Publisher('~gps_heading', PoseStamped, queue_size=1) 
    
    rospy.spin()
