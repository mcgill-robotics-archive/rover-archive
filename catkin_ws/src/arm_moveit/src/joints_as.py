#!/usr/bin/env python
"""Module grouping all the action servers for the joints."""

import rospy
import actionlib
from std_msgs.msg import Float64
from control_msgs.msg import FollowJointTrajectoryAction, JointControllerState


class JointActionServer(object):
    """Action server for a joint, links MoveIt! to the controllers."""

    def __init__(self, joint_name):

        # Set default position to 0.0, it get updated to the real value as
        # soon as a message is published to the state_topic.
        self.current_position = 0.0
        self.joint_name = joint_name

        action_name, command_topic, state_topic = joint_name_helper(joint_name)

        self.server = actionlib.SimpleActionServer(
            action_name, FollowJointTrajectoryAction,
            self._execute, auto_start=False)

        self.position_publisher = rospy.Publisher(
            command_topic, Float64, queue_size=1)

        self.position_subscriber = rospy.Subscriber(
            state_topic, JointControllerState,
            self._update_current_position, queue_size=1)

    def _update_current_position(self, state):
        self.current_position = state.process_value

    def _execute(self, goal):
        point_index = 0
        message = Float64()

        # Go through each point in the trajectory one by one
        for point in goal.trajectory.points:
            message.data = point.positions[0]
            self.position_publisher.publish(message)

            # Wait until the joint position is 0.001 or less from the target
            while(abs(message.data - self.current_position) < 0.001):
                pass

            rospy.logdebug(self.joint_name + " : Point " + str(point_index + 1)
                           + " at " + str(message.data) + " reached.")

            point_index += 1

        rospy.loginfo(self.joint_name + " : Goal at " + str(message.data)
                      + " reached.")
        self.server.set_succeeded()

    def _start(self):
        self.server.start()


def joint_name_helper(joint_name):
    """Create the strings for action name, command topic, and state topic."""
    action_name = joint_name + "_as"
    command_topic = "/arm/" + joint_name + "_position_controller/command"
    state_topic = "/arm/" + joint_name + "_position_controller/state"

    return action_name, command_topic, state_topic


def main():
    rospy.init_node("joints_as")

    joint_names = ["base_pitch", "base_yaw", "elbow_pitch",
                   "elbow_roll", "wrist_pitch", "wrist_roll"]

    for joint in joint_names:
        action_server = JointActionServer(joint)
        action_server._start()

    rospy.spin()


if __name__ == '__main__':
    main()
