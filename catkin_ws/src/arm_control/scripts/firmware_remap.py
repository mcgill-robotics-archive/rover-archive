#!/usr/bin/env python

"""Map the input from HCI arm controller to motor commands for open loop."""

import rospy
from std_msgs.msg import Int32
from arm_control.msg import JointVelocities

ARM_SIGNS = {
    "base_yaw": 1,
    "base_pitch": 1,
    "elbow_left": 1,
    "elbow_right": -1,
    "wrist_left": 1,
    "wrist_right": -1
}

ARM_BASE_DRIVE_RATIO = 2000

ARM_RATIO = {
    "base_yaw": ARM_BASE_DRIVE_RATIO / 2,  # Base yaw is compensated down.
    "base_pitch": ARM_BASE_DRIVE_RATIO,
    "elbow_pitch": ARM_BASE_DRIVE_RATIO,
    "elbow_roll": ARM_BASE_DRIVE_RATIO * 1.5,
    "wrist_pitch": ARM_BASE_DRIVE_RATIO * 1.5,
    "wrist_roll": ARM_BASE_DRIVE_RATIO * 1.5
}

class ArmRemapper(object):

    def update_state(self, joint_vel):
        self._joint_velocity = joint_vel
        self._joint_velocity_received = True

    def perform_remap(self):
        """Map the Joint Velocities onto motor commands."""
        motor_value = Int32()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self._joint_velocity_received:
                vel = self._joint_velocity

                # Shoulder Yaw
                signed_base_yaw = ARM_SIGNS["base_yaw"] * vel.base_yaw
                motor_value.data = signed_base_yaw * ARM_RATIO["base_yaw"]

                self._shoulder_yaw_pub.publish(motor_value)

                # Shoulder Pitch
                signed_base_pitch = ARM_SIGNS["base_pitch"] * vel.base_pitch
                motor_value.data = signed_base_pitch * ARM_RATIO["base_pitch"]

                self._shoulder_pitch_pub.publish(motor_value)

                # Elbow Diff Right Side
                scaled_d1_pitch = vel.diff_1_pitch * ARM_RATIO["elbow_pitch"]
                scaled_d1_roll = vel.diff_1_roll * ARM_RATIO["elbow_roll"]
                elbow_right = scaled_d1_roll - scaled_d1_pitch
                motor_value.data = elbow_right * ARM_SIGNS["elbow_right"]

                self._elbow_diff1_pub.publish(motor_value)

                # Elbow Diff Left Side
                elbow_left = -1 * (scaled_d1_pitch + scaled_d1_roll)
                motor_value.data = elbow_left * ARM_SIGNS["elbow_left"]

                self._elbow_diff2_pub.publish(motor_value)

                # Wrist Diff Right Side
		scaled_d2_pitch = vel.diff_2_pitch * ARM_RATIO["wrist_pitch"]
		scaled_d2_roll = vel.diff_2_roll * ARM_RATIO["wrist_roll"]
                wrist_right = scaled_d2_pitch + scaled_d2_roll
                motor_value.data = wrist_right * ARM_SIGNS["wrist_right"]

                self._wrist_diff1_pub.publish(motor_value)

                # Wrist Diff Left Side
                wrist_left = scaled_d2_pitch - scaled_d2_roll
                motor_value.data = wrist_left * ARM_SIGNS["wrist_left"]

                self._wrist_diff2_pub.publish(motor_value)

            rate.sleep()

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
