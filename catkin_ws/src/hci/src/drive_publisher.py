#!/usr/bin/env python
import rospy
from enum import IntEnum

from drive_control.msg import DriveCommand
from rover_common.msg import MotorControllerMode

## Group all the drive related ros operations 
#
class DrivePublisher(object):
    ## Constructor
    #
    # Registers the ROS publishers for general speed commands and controller mode.
    # When initialisation completes, a new timer is launched and published the 
    # drive instructions periodically.
    def __init__(self):
        self.speed_linear = 0
        self.speed_angular = 0
        self.steering_condition = SteeringCondition.Ackerman
        self.motor_controller_mode = MotorControllerTypeMode.OpenLoop
        self.enable = False

        ## Publisher object for drive commands
        self.command_publisher = rospy.Publisher(
            "/drive_command", DriveCommand, queue_size=1)
        ## Publisher object for motor controller mode
        self.controller_mode_publisher = rospy.Publisher(
            "/drive_controller_mode", MotorControllerMode, queue_size=1)

        ## Timer which publishes drive instructions at 10 hz (0.1 s)
        self.timer = rospy.Timer(rospy.Duration(secs=0, nsecs=100000000), self.run)
        
    ## Generate and send the message
    #
    # Builds the command message with the internally stored command information
    # and calls the publisher with the new message.
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

    ## Change the internal speed recorded for the next message 
    #
    # @param linear The new desired global robot linear velocity 
    #
    # @param angular The new global angular velocity
    def set_speed(self, linear, angular):
        self.speed_linear = linear / 2.0
        self.speed_angular = angular

    ## Sets the new desired steering condition for the angle calculation
    #
    # @param condition The steering condition from the SteeringCondition enum
    #
    def set_steering_condition(self, condition):
        self.steering_condition = condition

    ## Change the maxon motor controller mode 
    #
    # @param mode The controller mode from the MotorControllerTypeMode enum
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

    ## Change the enable flag for the next message
    #
    # @param enable The new enable flag
    def set_enable(self, enable):
        self.enable = enable

## Enumerated class for easier passing of the current motor controller mode
class MotorControllerTypeMode(IntEnum):
    OpenLoop = 3
    SlowSpeed = 0
    MediumSpeed = 1
    HighSpeed = 2

## Enumerated class for better passing of the steering mode
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
