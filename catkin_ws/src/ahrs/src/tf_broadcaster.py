#!/usr/bin/env python

import rospy

from tf.transformations import quaternion_multiply
import tf
import math
from ahrs.msg import AhrsStdMsg
from ahrs.srv import *


class AhrsTfBroadcaster:
    def __init__(self):
        rospy.init_node('tf_broadcaster')
        self.sub = rospy.Subscriber("ahrs_status",
                                    AhrsStdMsg, self.handle_ahrs_msg)

        self.center_serv = rospy.Service('center_ahrs_frame',
                                         CenterWorldFrame, self.handle_service)

        self.br = tf.TransformBroadcaster()
        self.center_position = (0, 0, 0)
        self.last_tf = None
        self.first = True
        rospy.loginfo("tf_broadcaster initialized, waiting for messages")

    def run(self):
        rospy.spin()
        rospy.loginfo("tr_broadcaster exited")

    def handle_service(self, req):
        if self.last_tf is not None:
            self.center_position = (
                self.last_tf.x,
                self.last_tf.y,
                self.last_tf.z)
        return CenterWorldFrameResponse(True)
        pass

    def handle_ahrs_msg(self, message):
        if self.first:
            rospy.loginfo("Received first message, tf are ready")

        quat = (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quat)
        euler_fixed = (euler[0], -1 * euler[1], -1 * euler[2])

        self.br.sendTransform((0.5, 0, 0.4), (0, 0, 0, 1),
                              rospy.Time.now(),
                              "ahrs_position", "base_link")

        # This adjustment makes the local frame for the ahrs conform with
        # the (forward, left, up) standard for local robot coordinate frames.
        # The AHRS is by default (forward, right, down).
        local_orientation_adjustment = (1, 0, 0, 0)

        self.br.sendTransform((0, 0, 0), local_orientation_adjustment,
                              rospy.Time.now(), "local_ahrs_orientation",
                              "ahrs_position")

        # This adjustment makes the global frame for the ahrs conform with the
        # east, north, up standard for global robot coordinate frames.
        # The AHRS is by default (north, east, down).
        sqrt2_2 = math.sqrt(2) / 2.0
        global_orientation_adjustment = (0, sqrt2_2, -sqrt2_2, 0)
        self.br.sendTransform((0, 0, 0), global_orientation_adjustment,
                              rospy.Time.now(), "global_ahrs_orientation",
                              "ahrs_position")

        self.last_tf = message.pose.pose.position


if __name__ == "__main__":
    tf_broadcaster = AhrsTfBroadcaster()
    tf_broadcaster.run()
