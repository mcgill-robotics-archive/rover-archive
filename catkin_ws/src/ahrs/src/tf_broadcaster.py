#!/usr/bin/env python

import rospy

from tf.transformations import quaternion_multiply
import tf
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
        if self.first :
            rospy.loginfo("Received first message, tf are ready")

        self.br.sendTransform((
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z),
            (0, 0, 0, 1), rospy.Time.now(), "ahrs_position", "world")

        quat = (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w)

        multiply = quaternion_multiply((1, 0, 0, 0), quat)

        self.br.sendTransform((0, 0, 0), multiply,
                              rospy.Time.now(), "ahrs_orientation", "ahrs_position")

        self.last_tf = message.pose.pose.position

        self.br.sendTransform(self.center_position, (0, 0, 0, 1),
                              rospy.Time.now(), "center", "world")
        self.br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "laser", "ahrs_orientation")


if __name__ == "__main__":
    tf_broadcaster = AhrsTfBroadcaster()
    tf_broadcaster.run()
