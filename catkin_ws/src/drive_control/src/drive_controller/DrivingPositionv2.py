#!/usr/bin/env python

#from drive_control.msg import WheelCommand
import rospy, time

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


# private var??
# clarify the angle value

class Wheel:
    def __init__(self, name):
        self.name = name
        self.angle = 0
        self.angVel = 0
        #self.linVel = 0

        # wheel radius in meter (diameter 8.625 in)
        WHEEL_RADIUS = 0.1095;
    
    def getTangentialVelocity(self):
          tanVel = WHEEL_RADIUS * angVel;
          return tanVel
    
    def getTangentialVelocityVector(self):
          tanVel = WHEEL_RADIUS * angVel;
          tanVelX = 
          tanVelY =
          tanVelVector = []
          return tanVelVector

class DrivingPosition:
    def __init__(self):

        self.wheels = [
            # front right wheel
            wheel('fr'),
            # back right wheel
            wheel('br'),
            # back left wheel
            wheel('bl'),
            # front left wheel
            wheel('fl')
        ]

	    self.robotPos = [0.0,0.0]
        self.start_time = time.clock()



    def update_x_pos(self):

        time.clock() - start_time


	    # For every wheel, compute the Vx and Vy
	    for wheel in self.wheels:
            wheel.getLinearVelocity()
            data['linVel'] = WHEEL_RADIUS * data['angVel']
        
	
    def update_robot_pos(self):

        self.robotPos = [posx,posy]











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

