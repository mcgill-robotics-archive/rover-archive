#!/usr/bin/env python
"""Module grouping all the action servers for the joints."""

import rospy
import actionlib
from std_msgs.msg import Float64
from control_msgs.msg import FollowJointTrajectoryAction


class JointActionServer(object):
    """Action server for a joint, links MoveIt! to the controllers."""

    in_motion_tolerance = 0.1
    final_position_tolerance = 0.02

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
            state_topic, Float64,
            self._update_current_position, queue_size=1)

    def _update_current_position(self, state):
        self.current_position = state.data

    def _check_if_point_reached(self, target, is_last):

        point_reached = False

        # For an intermediary point, we allow a fairly big error
        # since we do not want to stop mid-trajectory to reach it.
        # However, if we're going to the last point, we want a
        # much higher precision.
        tolerance = self.final_position_tolerance if is_last else self.in_motion_tolerance

        if abs(target - self.current_position) < tolerance:
            if is_last:
                # Check that we remain in the tolerance zone for 0.1 second
                # NOTE: For very small movements, MoveIt won't give us a lot
                # of time to reach the target, maybe these sleeps will end up
                # taking too much time. We'll see. -ASauvestre
                for i in range(10):
                    check_position_rate = rospy.Rate(100) # 100 Hz Loop
                    check_position_rate.sleep()

                    if abs(target - self.current_position) > tolerance:
                        return False # Early out to avoid sleeping any longer

            point_reached = True

        return point_reached

    def _execute(self, goal):
        point_index = 0
        last_point_index = len(goal.trajectory.points) - 1
        message = Float64()

        should_quit = False

        # Go through each point in the trajectory one by one
        for point in goal.trajectory.points:
            message.data = point.positions[0]

            is_last = (point_index == last_point_index)

            point_reached = False

            # Send the message until we reach the target point
            while not point_reached:
                # Check if we were preempted/cancelled
                if self.server.is_preempt_requested():
                    status = self.server.current_goal.status_tracker.status.status
                    rospy.loginfo("Preempting action server of joint " + self.joint_name + ", current status is: " + str(status))
                    self.server.set_preempted()
                    should_quit = True;
                    break

                publishing_rate = rospy.Rate(1000) # 1000Hz loop
                self.position_publisher.publish(message)

                point_reached = self._check_if_point_reached(point.positions[0], is_last)

                publishing_rate.sleep()

            rospy.logdebug(self.joint_name + " : Point " + str(point_index + 1)
                           + " at " + str(message.data) + " reached.")

            point_index += 1

            if should_quit:
                break

        if not should_quit:
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
    rospy.init_node("armexecute_kinematic_path")

    joint_names = ["base_pitch", "base_yaw", "elbow_pitch",
                   "elbow_roll", "wrist_pitch", "wrist_roll"]

    for joint in joint_names:
        action_server = JointActionServer(joint)
        action_server._start()

    rospy.spin()


if __name__ == '__main__':
    main()
