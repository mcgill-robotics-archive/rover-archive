#!/usr/bin/env python

from drive_control.msg import WheelCommand
import rospy

__author__ = 'David Lavoie-Boutin'

## Class WheelOutput is used as a structure to agglomerate the output values for the wheel velocities and angles.
#
# The member naming follows the following standard:
#
# front   : f
# middle  : m
# back    : b
#
# left    : l
# right   : r
#
# velocity        : v
# steering angle  : sa
#
# Combine one of each group and form the front-left-steering-angle: flsa
#
class WheelOutputData:
    def __init__(self):
        self.flsa = 0  #: front left steering angle
        self.frsa = 0  #: front right steering angle
        self.blsa = 0  #: back left steering angle
        self.brsa = 0  #: back right steering angle

        self.flv = 0  #: front left velocity
        self.frv = 0  #: front right velocity
        self.mlv = 0  #: middle left velocity
        self.mrv = 0  #: middle right velocity
        self.blv = 0  #: back left velocity
        self.brv = 0  #: back right velocity

    ## Simple method to reset the wheel velocities to zero
    def set_velocity_zero(self):
        self.flv = 0  #: front left velocity
        self.frv = 0  #: front right velocity
        self.mlv = 0  #: middle left velocity
        self.mrv = 0  #: middle right velocity
        self.blv = 0  #: back left velocity
        self.brv = 0  #: back right velocity

    def set_angle_zero(self):
        self.flsa = 0
        self.frsa = 0
        self.blsa = 0
        self.brsa = 0

    ## This member function creates a proper output ros message using the values contained in this class.
    #
    # @return ROS message WheelCommand from the member values ready to be published
    #
    def create_message(self):

        command = WheelCommand()
        command.flsa = self.flsa
        command.frsa = self.frsa
        command.blsa = self.blsa
        command.brsa = self.brsa
        command.flv = self.flv * 11
        command.frv = self.frv * 11
        command.mlv = self.mlv * 11
        command.mrv = self.mrv * 11
        command.blv = self.blv * 11
        command.brv = self.brv * 11

        return command

    ##Method to quickly set the values of all the class members
    #
    # @param data List of all the values to be set in the proper order
    def set_data(self, data):
        try:
            self.flsa = data[0]
            self.frsa = data[1]
            self.blsa = data[2]
            self.brsa = data[3]
            self.flv = data[4]
            self.frv = data[5]
            self.mlv = data[6]
            self.mrv = data[7]
            self.blv = data[8]
            self.brv = data[9]

        except KeyError:
            rospy.logerr("Bad data set received")

