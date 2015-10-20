#!/usr/bin/env python

import rospy

import tf
from ahrs.msg import AhrsStdMsg
from ahrs.srv import *

class AhrsTfBroadcaster:
    def __init__ (self):
        rospy.init_node('tf_broadcaster')
        self.sub = rospy.Subscriber("ahrs_status",
                AhrsStdMsg, self.handle_ahrs_msg) 
        
        self.center_serv = rospy.Service('center_ahrs_frame', 
                CenterWorldFrame, self.hander_service)

        self.br = tf.TransformBroadcaster()
        self.center_position = (0,0,0)
        self.last_tf = None

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()
        pass
    
    def hander_service(self, req):
        if self.last_tf is not None:
            self.center_position = (
                    self.last_tf.x,
                    self.last_tf.y,
                    self.last_tf.z)
        return CenterWorldFrameResponse(True)
        pass

    def handle_ahrs_msg(self, message):
        self.br.sendTransform((
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z),
            (0,0,0,1),rospy.Time.now(), "ahrs_position", "world")
        
        self.br.sendTransform((0,0,0),(
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w),
            rospy.Time.now(), "ahrs_orientation", "ahrs_position")

        self.last_tf = message.pose.pose.position

        self.br.sendTransform(self.center_position, (0,0,0,1), 
                rospy.Time.now(), "center", "world")

if __name__ == "__main__":
    tf_broadcaster = AhrsTfBroadcaster()
    tf_broadcaster.run()

