#!/usr/bin/env python
import rospy
import std_msgs
import math
import trajectory_calculator
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from drive_control.msg import DriveCommand
from navigation.srv import Reset, ResetResponse

class NavNode(object):

    TOLERANCE = 0.5

    def __init__(self):
        rospy.init_node("nav_node")
        self.publisher = rospy.Publisher('navigation/drive_command',
                                         DriveCommand,
                                         queue_size = 1)

        # A subscriber which subscribeds to the (x,y) coordinate pair which is
        # the position of the goal relative to the robot.
        # Internal state: self.goal = (x, y)
        self.subscriber = rospy.Subscriber("navigation/goal_coordinates",
                                           Point, self.callback)
        self.goal = Point()
        self.drive_command = DriveCommand()
        reset_server = rospy.Service('reset', Reset, self.reset)

    def callback(self, data):
        # We need to update the internal goal state.
        self.goal = data

        # If we don't have to go far then don't go anywhere
        if math.sqrt(self.goal.x ** 2 + self.goal.y ** 2) < NavNode.TOLERANCE:
            self.drive_command = DriveCommand()
            return

        # Update the trajectory circle's center
        center = trajectory_calculator.calculate_center_of_rotation(
            (self.goal.x, self.goal.y))
        # if the angle between the rover and the goal is >60 then need to
        # point steer until we are within this threshold

        # find the angle between the goal and the rover
        goal_angle = math.degrees(math.atan2(self.goal.y, self.goal.x))
        # want to rotate left if the goal angle is past 60 degrees
        if(goal_angle >= 1000):
            # now need to send point steering commands
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.5
            # now we need to populate the drive command
            self.drive_command = DriveCommand()
            self.drive_command.motion_enable = True
            self.drive_command.motion_pointsteer = True
            self.drive_command.velocity_command = twist
        # want to rotate right if the goal angle is past -60 degrees
        elif(goal_angle <= -10000):
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = -0.5
            # now to populate the drive command
            self.drive_command = DriveCommand()
            self.drive_command.motion_enable = True
            self.drive_command.motion_pointsteer = True
            self.drive_command.velocity_command = twist
        else:
            # Update the motor commands required to get there.
            radius = center[1]
            twist = Twist()
            twist.linear.x = 1.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            if(self.goal.y > 0):
                twist.angular.z = 1.0 / radius
            elif(self.goal.y < 0):
                twist.angular.z = -1.0 / radius
            else:
                twist.angular.z = 0.0
            #need to send commands to get there
            self.drive_command = DriveCommand()
            self.drive_command.velocity_command = twist
            self.drive_command.motion_ackerman = True
            self.drive_command.motion_enable = True

    def reset(self, reset_input):
        #set everything to zero
        self.drive_command.motion_enable = False
        self.drive_command.motion_ackerman = False
        self.drive_command.velocity_command.linear.x = 0.0
        self.drive_command.velocity_command.angular.z = 0.0
        return ResetResponse(True)


if __name__ == "__main__":
    navNode = NavNode()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        # publish the internal drive command state
        navNode.publisher.publish(navNode.drive_command)
        r.sleep()
