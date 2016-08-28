#!/usr/bin/env python
import rospy

import tf
import turtlesim.msg

def handle_turtle_pose(msg, turtlename):
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        br.sendTransform((0,1,2), tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                        "robot",
                        "world")
        rate.sleep()

