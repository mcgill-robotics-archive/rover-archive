#!/usr/bin/env python

import rospy
import tf
from tf.transformations import quaternion_from_euler
from arm_control.msg import JointPosition


class ArmTf(object):

    baseFrame = "/robot"
    baseYawFrame = "/arm/base_yaw"
    pitch1Frame = "/arm/base_pitch"
    pitch2Frame = "/arm/pitch2"
    roll1Frame = "/arm/roll1"
    pitch3Frame = "/arm/pitch3"
    roll2Frame = "/arm/roll2"

    armPosition = 0.10
    pitch1offset = 0.02
    pitch2offset = 0.10
    roll1offset = 0.02
    pitch3offset = 0.10
    roll2offset = 0.02

    def __init__(self):
        rospy.init_node("tf_desired")
        self.sub = rospy.Subscriber("desired_joint_angles", JointPosition, self.callback)
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.loginfo("Desired TF broadcaster initialized")
        rospy.spin()
        rospy.loginfo("Exiting")

    def callback(self, message):
        base_yaw_quat = tf.transformations.quaternion_from_euler(message.base_yaw, 0, 0, 'rzyx')
        base_pitch_quat = tf.transformations.quaternion_from_euler(0, message.base_pitch, 0, 'rzyx')
        diff_1_pitch_quat = tf.transformations.quaternion_from_euler(0, message.diff_1_pitch, 0, 'rzyx')
        diff_2_pitch_quat = tf.transformations.quaternion_from_euler(0, message.diff_2_pitch, 0, 'rzyx')
        diff_1_roll_quat = tf.transformations.quaternion_from_euler(0, 0, message.diff_1_roll, 'rzyx')
        diff_2_roll_quat = tf.transformations.quaternion_from_euler(0, 0, message.diff_2_roll, 'rzyx')

        self.tf_broadcaster.sendTransform((self.armPosition, 0, 0), base_yaw_quat, rospy.Time.now(), self.baseYawFrame, self.baseFrame)
        self.tf_broadcaster.sendTransform((0, 0, self.pitch1offset), base_pitch_quat, rospy.Time.now(), self.pitch1Frame, self.baseYawFrame)
        self.tf_broadcaster.sendTransform((self.pitch2offset, 0, 0), diff_1_pitch_quat, rospy.Time.now(), self.pitch2Frame, self.pitch1Frame)
        self.tf_broadcaster.sendTransform((self.roll1offset, 0, 0), diff_1_roll_quat, rospy.Time.now(), self.roll1Frame, self.pitch2Frame)
        self.tf_broadcaster.sendTransform((self.pitch3offset, 0, 0), diff_2_pitch_quat, rospy.Time.now(), self.pitch3Frame, self.roll1Frame)
        self.tf_broadcaster.sendTransform((self.roll2offset, 0, 0), diff_2_roll_quat, rospy.Time.now(), self.roll2Frame, self.pitch3Frame)

if __name__ == "__main__" :
    tf = ArmTf()
