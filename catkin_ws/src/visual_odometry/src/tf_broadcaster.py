#!/usr/bin/env python

import rospy
import tf

def handle_msg():
    br = tf.TransformBroadcaster()
    br.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), "axis_camera", "base_link")
    
if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rospy.spin()
    
