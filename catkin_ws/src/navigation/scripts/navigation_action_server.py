#!/usr/bin/env python

"""Action server which takes in a Odometry message and tries to get there."""

import math
import rospy
import tf
import actionlib
import LatLon
import numpy as np
from ahrs.msg import AhrsStdMsg
from sensor_msgs.msg import NavSatFix
from navigation.srv import Reset
from navigation.msg import NavToWaypointAction, NavToWaypointFeedback
from geometry_msgs.msg import Point


class NavToWaypointServer(object):
    """Action server for navigating to a point in the map frame."""
    def __init__(self):
        self._action_name = "nav_to_waypoint"
        self._server = actionlib.SimpleActionServer(
            self._action_name, NavToWaypointAction, self._execute, False)
        rospy.Subscriber("/ahrs_manager/gps", NavSatFix,
                         self._update_current_positon)

        rospy.Subscriber("/ahrs/ahrs_status", AhrsStdMsg,
                         self._update_current_positon)

        self._publisher = rospy.Publisher('/navigation/goal_coordinates',
                                          Point, queue_size=1)
        self._feedback = NavToWaypointFeedback()

        # The relative goal is a point in the robots frame.
        self._relative_goal = Point()
        self._yaw = 0.0

        rospy.wait_for_service('reset')

        self._server.start()

    def _update_yaw(self, ahrs_status):
        self._yaw = ahrs_status.gps.heading * math.pi / 180.0

    def _update_current_positon(self, nav_sat_fix):
        """This subscriber simply sets the current pose based on odometry."""
        self._feedback.feedback = nav_sat_fix

    def reset_client(self, reset_input):
        rospy.wait_for_service('reset')
        try:
            reset = rospy.ServiceProxy('reset', Reset)
            did_reset = reset(reset_input)
            return did_reset
        except rospy.ServiceException as e:
            print("Service call failed %s"%e)

    def _calculate_goal(self, current_coordinates, goal_coordinates):
        current_coordinates_ll = LatLon(current_coordinates)
        goal_coordinates_ll = LatLon(goal_coordinates)

        distance = current_coordinates_ll.distance(goal_coordinates_ll) * 1000

        required_heading_ll_frame = current_coordinates_ll.heading_initial(
            goal_coordinates_ll) * math.pi / 360.0

        required_heading = (math.pi / 2.0) - required_heading_ll_frame

        theta = required_heading - self._yaw

        self._relative_goal = Point(np.cos(theta) * relative_distance,
                                    np.sin(theta) * relative_distance,
                                    0)

    def _execute(self, goal):
        """Entry point for action server."""
        rate = rospy.Rate(10)

        goal_coordinates = (goal.goal.latitude, goal.goal.longitude)

        # The assumption is we will always be preempted by the client.
        while not self._server.is_preempt_requested() \
              and not rospy.is_shutdown():
            current_coordinates = (self._feedback.feedback.latitude,
                                  self._feedback.feedback.longitude)
            self._calculate_goal(current_coordinates, goal_coordinates)
            self._publisher.publish(self._relative_goal)
            self._server.publish_feedback(self._feedback)
            rate.sleep()

        if not self._server.is_new_goal_available():
            # Reset the drive commands to not move.
            self.reset_client(True)
            rospy.loginfo("%s: Aborted", self._action_name)
            self._server.set_aborted()
        else:
            # Preempt has been requested, exit.
            rospy.loginfo("%s: Preempted", self._action_name)
            self._server.set_preempted()


if __name__ == "__main__":
    rospy.init_node("nav_to_waypoint_server")
    NavToWaypointServer()
    rospy.spin()
