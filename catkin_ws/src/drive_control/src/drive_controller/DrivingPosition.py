#!/usr/bin/env python

#from drive_control.msg import WheelCommand
import rospy, time, math

__author__ = 'Laetitia Fesselier'

# private var??

class Wheel:
    def __init__(self, name):
        self.name = name
        self.angle = 0
        self.angVel = 0

        # wheel radius in meter (diameter 8.625 in)
        WHEEL_RADIUS = 0.1095;
    
    def getLinearVelocity(self):
          tanVel = WHEEL_RADIUS * angVel
          return tanVel
    
    def getLinearVelocityVector(self):
          tanVel = getTangentialVelocity()
          tanVelX = tanVel * math.sin(self.angle)
          tanVelY = tanVel * math.cos(self.angle)
          tanVelVector = [tanVelX,tanVelY]
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
        self.time = time.clock()      
        self.timeSpan

    def update_wheels(self):
        # update time for compute the velocity

        newTime = time.clock()
        self.timeSpan = newTime - time
        self.time = newTime

        # CODE TO GET NEW VEL - call a topic and update for each
        for wheel in self.wheels:
            newAngle = 0.3
            newAngVel = 2

            wheel['angle'] = newAngle
            wheel['angVel'] = newAngVel
        
    def update_robot_pos(self):
	    # For every wheel, compute the Vx and Vy
	    for wheel in self.wheels:
            tanVelVector = wheel.getTangentialVelocityVector()
            robotTanVelX += tanVelVector[0]
            robotTanVelY += tanVelVector[1]
        
        robotTanVelX = robotTanVelX / len(self.wheels)
        robotTanVelY = robotTanVelY / len(self.wheels)
        
        robotPosX += robotTanVelX * timeSpan
        robotPosY += robotTanVelY * timeSpan

	    self.robotPos = [robotPosX,robotPosY]
        return self.robotPos


    # We need to compute the angle 

