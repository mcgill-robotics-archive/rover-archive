#!/usr/bin/env python

import rospy
import rosbag
import actionlib
from navigation.msg import NavToWaypointAction, NavToWaypointGoal
from math import sqrt
import LatLon

class NavigationClient:
    # Number of odometry msg to ignore between goals
    _GOAL_DECIMATION = 100
    # Euclidian distance between goal and current location to register goal
    # reached and to send new goal
    _GOAL_ACCEPT_RANGE = 5.0
    # Timeout in seconds to warn  no feedback received
    _FEEDBACK_TIMEOUT = 2
    # Timeout in seconds to warn goal not reached in time
    _GOAL_TIMEOUT_WARNING = 20

    def __init__(self):
        '''Class initializer.
        '''
        self._nh = rospy.init_node("nav_client")
        rospy.loginfo('Starting navigation by demonstraion client...')
        self._bag_name = rospy.get_param('~bag_name', 'demonstration.bag')
        self._topic_name = rospy.get_param('~topic_name',
                '/ahrs/ahrs_status')

        rospy.loginfo('Opening {0:s}...'.format(self._bag_name))
        self._bag = rosbag.Bag(self._bag_name)
        self._msg_count = self._bag.get_type_and_topic_info()[1][
                self._topic_name].message_count
        self._current_index = 0
        self._messages = self._bag.read_messages(topics=self._topic_name)
        rospy.loginfo('Topic {0:s} found with {1:d} message(s)'.format(
                self._topic_name, self._msg_count))

        rospy.loginfo('Starting action client...')
        self._client = actionlib.SimpleActionClient("nav_to_waypoint",
                NavToWaypointAction)

        rospy.loginfo('Waiting for action server...')
        self._client.wait_for_server()

        # Setup the first goal
        self._diagnostic_rate = rospy.Rate(1)
        self._current_goal = next(self._messages).message
        self._send_next_goal()
        self._last_feedback = rospy.Time.now()
        self._is_goal_final = False

    def _send_next_goal(self):
        '''Function to send the current goal to the action server and find the
        next goal to send.
        '''
        self._goal_send_time = rospy.Time.now()
        self._client.stop_tracking_goal()
        rospy.loginfo('Sending new goal...')
        rospy.loginfo('Current goal: {0:d}, total goals: {1:d}'.format(
                self._current_index, self._msg_count))

        action_goal = NavToWaypointGoal()
        action_goal.goal = self._current_goal

        self._client.send_goal(action_goal,
                feedback_cb=self._feedback_callback)
        for x in range(self._GOAL_DECIMATION):
            try:
                self._current_goal = next(self._messages).message
                self._current_index += 1
            except StopIteration:
                self._is_goal_final = True
                break

    def _feedback_callback(self, feedback):
        '''Function to handle feedback from the action server.
        '''

        self._last_feedback = rospy.Time.now()
        distance = self._get_distance_to_goal(self._current_goal,
                                             feedback.feedback)

        if distance < self._GOAL_ACCEPT_RANGE:
            rospy.loginfo('Navigated within acceptable range...')

            if self._is_goal_final:
                self._client.cancel_all_goals()
                rospy.signal_shutdown('Final goal reached!!!')
            else:
                self._send_next_goal()
        else:
            rospy.loginfo_throttle(2, 'Distance to current goal' +
                   ' : {0:.2f}'.format(distance))

    def _get_distance_to_goal(self, goal, feedback):
        '''Get the distance between the current goal odometry message and the
        feed back odometry message.

        Args:
            goal: The current goal as odometry message.
            feedback: The current feedback as odometry message.
        '''
        goal_coord = LatLon(Latitude(goal.gps.latitude), 
                            Longitude(goal.gps.longitude))
        feedback_coord = LatLon(Latitude(feedback.gps.latitude), 
                                Longitude(feedback.gps.longitude))
        return goal_coord.distance(feedback_coord) * 1000

    def spin(self):
        '''Spin the node indefinitely and output some diagnostic information
        '''
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            if current_time - self._last_feedback > rospy.Duration(
                    self._FEEDBACK_TIMEOUT):
                rospy.logwarn('No feedback received,' +
                        ' check navigation action server...')
                self._last_feedback = current_time

            if current_time - self._goal_send_time > rospy.Duration(
                    self._GOAL_TIMEOUT_WARNING):
                rospy.logwarn('Server unable to navigate to goal within set' +
                        ' time, keep trying...')
                self._goal_send_time = current_time - rospy.Duration(5)
            self._diagnostic_rate.sleep()
        rospy.loginfo("Node shutdown...")

if __name__ == "__main__":
    client = NavigationClient()
    client.spin()
