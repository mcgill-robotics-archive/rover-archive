__author__ = 'David'

import rospy
from geometry_msgs.msg import Twist

import tf


class ArmControl:
    def __init__(self):

        rospy.init_node("arm_control")
        rospy.Subscriber("arm_command", Twist, self.command_callback)
        self.tf_listener = tf.TransformListener()
        pass

    def command_callback(self, message):
        
        (current_pos, current_rot) = self.tf_listener.lookupTransform('/robot', '/world', rospy.Time(0))
        target_pos = [current_pos[0] + message.linear.x / 100.0,
                      current_pos[1] + message.linear.y / 100.0,
                      current_pos[2] + message.linear.z / 100.0]
        change_rot = tf.transformations.quaternion_from_euler(message.angular.x / 100.0, message.angular.y / 100.0, message.angular.z / 100.0)

        target_rot = [current_rot[0] + change_rot[0],
                      current_rot[1] + change_rot[1],
                      current_rot[2] + change_rot[2],
                      current_rot[3] + change_rot[3]]

        print target_pos,target_rot

if __name__ == '__main__':
    ctl = ArmControl()
    rospy.spin()  