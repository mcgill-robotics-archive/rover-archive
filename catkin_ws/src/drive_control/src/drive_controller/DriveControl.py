#!/usr/bin/env python
import rospy
from Steering import Steering
from drive_control.msg import DriveCommand, WheelCommand
from std_msgs.msg import Bool

__author__ = 'David Lavoie-Boutin'


class DriveControl:
    def __init__(self):
        self.general_command = [0, 0]
        self.input_command = DriveCommand()
        self.steering = Steering()

        rospy.init_node("drive_controller")
        self.command_subscriber = rospy.Subscriber("/drive_command", DriveCommand, self.update_value_settings)
        self.command_publisher = rospy.Publisher("/wheel_command", WheelCommand, queue_size=1)
        self.moving_publisher = rospy.Publisher("/is_moving", Bool, queue_size = 1)
        self.verbose = rospy.get_param("~verbose", False)

        self.is_moving = False
        self.rotation = 0

    def update_value_settings(self, msg):
        self.general_command[0] = msg.velocity_command.linear.x
        self.general_command[1] = msg.velocity_command.angular.z

        self.is_moving = msg.motion_enable

        if msg.motion_ackerman:
            self.steering.steer(self.general_command[0], self.general_command[1])

        elif msg.motion_pointsteer:
            self.steering.pointTurn(self.general_command[1])

        elif msg.motion_translatory:
            self.steering.translationalMotion(self.general_command[0], self.general_command[1])

        elif msg.motion_skid:
            self.steering.skid_steer(self.general_command[0], self.general_command[1])

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.is_moving:
                self.steering.output_command.set_velocity_zero()

            if self.verbose:
                # TODO: log information to screen
                pass

            bool_msg = Bool()
            bool_msg.data = self.is_moving
            message = self.steering.output_command.create_message()
            self.command_publisher.publish(message)
            self.moving_publisher.publish(bool_msg)

            r.sleep()
        pass

    def close(self):
        pass


if __name__ == "__main__":
    rospy.loginfo("Starting drive control program")
    drive_control = DriveControl()
    rospy.loginfo("Drive control was properly initialized, entering main loop")
    drive_control.run()
    rospy.spin()
    pass
