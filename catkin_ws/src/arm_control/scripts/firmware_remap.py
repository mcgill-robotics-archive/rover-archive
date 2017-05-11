#!/usr/bin/env python

"""Map the input from HCI arm controller to motor commands for open loop."""

import rospy
from std_msgs.msg import Int32
from arm_control.msg import JointVelocities


class ArmRemapper(object):

    ARM_RATIO = 2000

    def update_state(self, joint_vel):
        self._joint_velocity = joint_vel
        self._joint_velocity_received = True

    def perform_remap(self):
        """Map the Joint Velocities onto motor commands."""
        motor_value = Int32()

        while not rospy.is_shutdown():
            if self._joint_velocity_received:
                vel = self._joint_velocity

                # Shoulder Yaw
                motor_value.data = vel.base_yaw * ArmRemapper.ARM_RATIO
                self._shoulder_yaw_pub.publish(motor_value)

                # Shoulder Pitch
                motor_value.data = vel.base_pitch * ArmRemapper.ARM_RATIO
                self._shoulder_pitch_pub.publish(motor_value)

                # Elbow Diff Right Side
                motor_value.data = ArmRemapper.ARM_RATIO * (vel.diff_1_pitch +
                                                            vel.diff_1_roll)
                self._elbow_diff1_pub.publish(motor_value)

                # Elbow Diff Left Side
                motor_value.data = ArmRemapper.ARM_RATIO * (vel.diff_1_pitch -
                                                            vel.diff_1_roll)
                self._elbow_diff2_pub.publish(motor_value)

                # Wrist Diff Right Side
                motor_value.data = ArmRemapper.ARM_RATIO * (vel.diff_2_pitch +
                                                            vel.diff_2_roll)
                self._wrist_diff1_pub.publish(motor_value)

                # Wrist Diff Left Side
                motor_value.data = ArmRemapper.ARM_RATIO * (vel.diff_2_pitch -
                                                            vel.diff_2_roll)
                self._wrist_diff2_pub.publish(motor_value)

    def __init__(self):
        """Initialize the ROS node and Subscribe to JointVelocities."""
        rospy.init_node("arm_control_remapper")

        self._joint_velocity_received = False

        rospy.Subscriber("david/joint_velocity", JointVelocities,
                         self.update_state)

        self._shoulder_pitch_pub = rospy.Publisher("motor_shoulder_a", Int32,
                                                   queue_size=1)

        self._shoulder_yaw_pub = rospy.Publisher("motor_shoulder_b", Int32,
                                                 queue_size=1)

        self._elbow_diff1_pub = rospy.Publisher("motor_elbow_a", Int32,
                                                queue_size=1)

        self._elbow_diff2_pub = rospy.Publisher("motor_elbow_b", Int32,
                                                queue_size=1)

        self._wrist_diff1_pub = rospy.Publisher("motor_wrist_a", Int32,
                                                queue_size=1)

        self._wrist_diff2_pub = rospy.Publisher("motor_wrist_b", Int32,
                                                queue_size=1)


if __name__ == "__main__":
    remapper = ArmRemapper()
    remapper.perform_remap()
