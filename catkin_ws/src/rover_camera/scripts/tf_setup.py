#!/usr/bin/env python
from math import pi

import rospy
import tf
import tf.transformations


class CamTfSetup(object):
    def __init__(self):
        rospy.init_node("cam_tf_broadcaster")

        self.br = tf.TransformBroadcaster()

    def run(self):
        while not rospy.is_shutdown():
            cam_left_tf_pos = (0, -0.40, 0)
            cam_left_tf_ori = tf.transformations.quaternion_from_euler(0, pi/2, -pi/2)

            cam_right_tf_pos = (0, 0.40, 0)
            cam_right_tf_ori = tf.transformations.quaternion_from_euler(0, pi/2, pi/2)

            cam_front_tf_pos = (0.50, 00, 0)
            cam_front_tf_ori =  tf.transformations.quaternion_from_euler(0, pi/2, 0)

            self.br.sendTransform(cam_left_tf_pos, cam_left_tf_ori,  rospy.Time.now(), "left_haz_frame", "ahrs_orientation")
            self.br.sendTransform(cam_right_tf_pos, cam_right_tf_ori, rospy.Time.now(), "right_haz_frame", "ahrs_orientation")
            self.br.sendTransform(cam_front_tf_pos, cam_front_tf_ori, rospy.Time.now(), "front_haz_frame", "ahrs_orientation")

        rospy.loginfo("tr_broadcaster exited")

if __name__ == "__main__":
    broad = CamTfSetup()
    broad.run()
