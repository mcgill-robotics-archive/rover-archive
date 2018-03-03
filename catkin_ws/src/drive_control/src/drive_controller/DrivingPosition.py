#!/usr/bin/env python

#from drive_control.msg import WheelCommand
import rospy

__author__ = 'Laetitia Fesselier'

## Class DrivingPosition is used as a structure to agglomerate the output values for the wheel velocities and angles.
# 
# The member naming follows the following standard:
# 
# front   : f
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

class Wheel:
    def __init__(self, name):
        self.name = name
        self.angle = 0
        self.angVel = 0
        self.linVel = 0
        self.vx = 0
        self.vy = 0

        # wheel radius in meter (diameter 8.625 in)
        WHEEL_RADIUS = 0.1095;

    def 



class DrivingPosition:
    def __init__(self):

        # Lae -> Adrien : Do we need the ids ? We can simplify this data stricture by removing the keys and let fr be 0, br be 1, ...
        # But less readable

        self.wheels = {
            # front right wheel
	        'fr': {
		        'angle': 0,
                'angVel': 0
                'linVel': 0
                'vx': 0
                'vy': 0
            },
            # back right wheel
            'br': {
		        'angle': 0,
                'angVel': 0,
                'linVel': 0,
                'vx': 0
                'vy': 0
            },
            # back left wheel
            'bl': {
		        'angle': 0,
                'angVel': 0,
                'linVel': 0
            },
            # front left wheel
            'fl': {
		        'angle': 0,
                'angVel': 0,
                'linVel': 0
            }
        }

	    self.xpos = 0;
	    self.ypos = 0;

        # wheel radius in meter (diameter 8.625 in)
        WHEEL_RADIUS = 0.1095;

    def update_x_pos(self):
	    # For every wheel, compute the Vx and Vy
	    for wheelId, data in self.wheels.iteritems():
            data['linVel'] = WHEEL_RADIUS * data['angVel']
        
	
	self.flsa
        self.frsa
        self.blsa
        self.brsa

        self.flv = 0  #: front left velocity
        self.frv = 0  #: front right velocity
        self.blv = 0  #: back left velocity
        self.brv = 0  #: back right velocity



	self.xpos = 
    

    ## Simple method to reset the wheel velocities to zero
    def set_velocity_zero(self):
        self.flv = 0  #: front left velocity
        self.frv = 0  #: front right velocity
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
        command.flv = self.flv * 7
        command.frv = self.frv * 7
        command.mlv = self.mlv * 7
        command.mrv = self.mrv * 7
        command.blv = self.blv * 7
        command.brv = self.brv * 7

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

