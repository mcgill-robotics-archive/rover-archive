#!/usr/bin/env python
import rospy
from enum import IntEnum

from drive_control.msg import DriveCommand
from rover_common.msg import MotorControllerMode


class DrivePublisher(object):
    def __init__(self):
        self.speed_linear = 0
        self.speed_angular = 0
        self.steering_condition = SteeringCondition.Ackerman
        self.motor_controller_mode = MotorControllerTypeMode.OpenLoop
        self.enable = False

        self.command_publisher = rospy.Publisher(
            "drive_command", DriveCommand, queue_size=1)
        self.controller_mode_publisher = rospy.Publisher(
            "drive_controller_mode", MotorControllerMode, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(secs=0, nsecs=100000000), self.run)
        
    def run(self, event):
        message = DriveCommand()
        message.velocity_command.linear.x = self.speed_linear
        message.velocity_command.angular.z = self.speed_angular
        message.motion_enable = self.enable

        if self.steering_condition == SteeringCondition.Ackerman:
            message.motion_ackerman = True
        elif self.steering_condition == SteeringCondition.Point:
            message.motion_pointsteer = True
        elif self.steering_condition == SteeringCondition.Translation:
            message.motion_translatory = True
        elif self.steering_condition == SteeringCondition.Skid:
            message.motion_skid = True
        elif self.steering_condition == SteeringCondition.Swerve:
            message.motion_swerve = True

        self.command_publisher.publish(message)

    def close(self):
        pass

    def set_speed(self, linear, angular):
        self.speed_linear = linear * 3
        self.speed_angular = angular

    def set_steering_condition(self, condition):
        self.steering_condition = condition

    def set_motor_controller_mode(self, mode):
        self.motor_controller_mode = mode

        message = MotorControllerMode()
        if self.motor_controller_mode == MotorControllerTypeMode.SlowSpeed:
            message.lowSpeed = True
        elif self.motor_controller_mode == MotorControllerTypeMode.MediumSpeed:
            message.medSpeed = True
        elif self.motor_controller_mode == MotorControllerTypeMode.HighSpeed:
            message.highSpeed = True
        elif self.motor_controller_mode == MotorControllerTypeMode.OpenLoop:
            message.openLoop = True

        self.controller_mode_publisher.publish(message)

    def set_enable(self, enable):
        self.enable = enable


class MotorControllerTypeMode(IntEnum):
    OpenLoop = 3
    SlowSpeed = 0
    MediumSpeed = 1
    HighSpeed = 2


class SteeringCondition(IntEnum):
    Ackerman = 0
    Point = 1
    Translation = 2
    Skid = 3
    Swerve = 4

if __name__ == '__main__':
    rospy.init_node("drive_publisher")
    ctl = DrivePublisher()
    rospy.spin()
