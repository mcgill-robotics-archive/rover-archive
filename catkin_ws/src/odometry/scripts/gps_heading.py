#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from ahrs.msg import AhrsStdMsg
from std_msgs.msg import Header
from numpy import pi
import tf

# Global Constants
DISCARD_COUNT = 200

# Global Variables
HEADING_COUNT = 0.0


def ahrs_callback(msg):
    '''Callback function for receiving ARHS message.

    Arg:
        msg: ROS msg object for the callback.
    '''
    global HEADING_COUNT

    # Create new message and header
    if msg.gps.FIX_3D:
        rospy.loginfo_throttle(60, "gps_heading: GPS 3D Fixed!")
        if HEADING_COUNT < DISCARD_COUNT:
            HEADING_COUNT += 1
            rospy.loginfo_throttle(
                1, 'gps_heading: Discarding first few messages:' +
                '{0:.2%}  Complete'.format(HEADING_COUNT / DISCARD_COUNT))
        else:
            rospy.loginfo_throttle(
                1, 'gps_heading: Current Heading:' +
                '{0:.2f} degree' .format(msg.gpsHeading.data / 100000.0))
            heading = msg.gpsHeading.data * pi / 180 / 100000
            q = tf.transformations.quaternion_from_euler(0, 0, heading)
            robot_pose = PoseWithCovarianceStamped()
            robot_pose.header = Header(
                frame_id=frame_id, stamp=rospy.Time.now())
            robot_pose.pose.pose.orientation.x = q[0]
            robot_pose.pose.pose.orientation.y = q[1]
            robot_pose.pose.pose.orientation.z = q[2]
            robot_pose.pose.pose.orientation.w = q[3]
            robot_pose.pose.covariance = [
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0.01]
            pub.publish(robot_pose)


if __name__ == '__main__':
    # Init the ROS node
    rospy.init_node('gps_heading')
    rospy.loginfo("GPS Heading node started")
    frame_id = rospy.get_param("~frame_id", "base_link")
    sub = rospy.Subscriber('/ahrs/ahrs_status', AhrsStdMsg,
                           ahrs_callback, queue_size=1)
    pub = rospy.Publisher('~gps_heading', PoseWithCovarianceStamped,
                          queue_size=1)

    rospy.spin()
