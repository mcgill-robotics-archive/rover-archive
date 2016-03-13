__author__ = 'David'

import rospy
from arm_control.msg import *

## Group the arm control functions in the same class 
#
class ArmPublisher:

    ## Constructor creates the ros publisher object and an empty message
    #
    def __init__(self):

        ## Publisher object vor velocities
        self.publisher = rospy.Publisher("arm_velocities", JointVelocities, queue_size=1)
        self.message = JointVelocities()

    ## Publish all joint velocities
    # 
    def publish_joint_vels(self, base_pitch, diff_1_pitch, diff_2_pitch, diff_1_roll, diff_2_roll, end_effector):
        self.message.base_pitch = base_pitch
        self.message.diff_1_pitch = diff_1_pitch
        self.message.diff_2_pitch = diff_2_pitch
        self.message.diff_1_roll = diff_1_roll
        self.message.diff_2_roll = diff_2_roll
        self.message.end_effector = end_effector

        self.publisher.publish(self.message)

    def publish_base_pitch(self, base_pitch):
        self.message.base_pitch = base_pitch
        self.message.diff_1_pitch = 0
        self.message.diff_2_pitch = 0
        self.message.diff_1_roll = 0
        self.message.diff_2_roll = 0
        self.message.end_effector = 0

        self.publisher.publish(self.message)


    def publish_diff_1(self, diff_1_pitch, diff_1_roll):
        self.message.base_pitch = 0
        self.message.diff_1_pitch = diff_1_pitch
        self.message.diff_2_pitch = 0
        self.message.diff_1_roll = diff_1_roll
        self.message.diff_2_roll = 0
        self.message.end_effector = 0

        self.publisher.publish(self.message)


    def publish_diff_2(self, diff_2_pitch, diff_2_roll):
        self.message.base_pitch = 0
        self.message.diff_2_pitch = diff_2_pitch
        self.message.diff_1_pitch = 0
        self.message.diff_2_roll = diff_2_roll
        self.message.diff_1_roll = 0
        self.message.end_effector = 0

        self.publisher.publish(self.message)


    def publish_end_effector(self, end_effector):
        self.message.base_pitch = 0
        self.message.diff_2_pitch = 0
        self.message.diff_1_pitch = 0
        self.message.diff_2_roll = 0
        self.message.diff_1_roll = 0
        self.message.end_effector = end_effector

        self.publisher.publish(self.message)

