#!/usr/bin/env python

import rospy
import math
from Utilities import *
from WheelOutputData import WheelOutputData

__author__ = 'David Lavoie-Boutin'


class Steering:
    """
    Steering class provides a simple interface to all the standard driving modes
    """
    def __init__(self):
        self.output_command = WheelOutputData()

        # distance between longitudinal axis and wheels[m]
        self.D = rospy.get_param('control/wh_distance_fr', 1.08 / 2.0)
        self.B = rospy.get_param('control/wh_base', 0.68 / 2.0)
        self.R = rospy.get_param('control/wh_radius', 0.1143)  #: wheel radius [m]
        self.W = rospy.get_param('control/wh_width', 0.15)  #: wheel width [m]

        self.mid_wh_offset = rospy.get_param('control/middle_wh_offset', 0.106)
        # angle on the wheels for point steering
        self.pointSteeringAngle = math.pi / 2 - math.atan(self.B / self.D)
        # radius from the wheels to the middle of the rover for point steering
        self.pointSteeringRadius = math.sqrt(self.D ** 2 + self.B ** 2)
        
        self.zero = 1e-10  # Offers protection against numbers very close to zero

        # minimum rhoMin (just in front of wheel)
        self.rhoMin = self.B + self.W / 2

        pass
    
    def stop(self):
        """
        Bring the 6 wheels to a stop, sets velocity 0 to all wheels
        """
        self.output_command.set_velocity_zero()
        self.output_command.set_angle_zero()

    def skid_steer(self, vBody, diff):
        """
        Use skid steering method to drive the robot

        All wheels point straight, steering angles are zero and rotation is due to the difference in wheel speed
        :param vBody: General body velocity
        :param diff: Rate of rotation
        """
        common_mode_scale = 10
        differential_mode_scale = 10

        common_mode = vBody * common_mode_scale
        differential_mode = diff * differential_mode_scale

        self.output_command.flv = common_mode + differential_mode
        self.output_command.frv = common_mode - differential_mode
        self.output_command.mlv = self.output_command.flv
        self.output_command.blv = self.output_command.flv
        self.output_command.mrv = self.output_command.frv
        self.output_command.brv = self.output_command.frv

    def steer(self, vBody, wBody):
        """
        Perform ackerman steering, following the proper ackerman steering condition

        This function maps joystick input to the proper steering angle of the four corner wheels
        (except for when wheels cannot accommodate)
        :param vBody: General body linear velocity
        (note that the rover cannot have angular velocity without linear velocity)
        :param wBody: Total body angular velocity (rate of turn)
        """
        # To be used in the final calculation of the velocity
        sign_v = sign(vBody)

        # angle of the wheels also depend on which way the rover is moving
        sign_w = sign(wBody)

        # absolute values used in the calculations
        vBody = abs(float(vBody))
        wBody = abs(float(wBody))

        # if robot not moving
        if abs(vBody) < self.zero:
            # indicates that no settings should change, robot
            # should just stop movement
            # in the future we might want to put a button to
            self.stop()

        # higher value of "zero" is so that when the user aims straight,
        # even if the joystick is a little bit off, the robot will go straight
        elif abs(wBody) < self.zero * 1e7:
            # straight velocity
            # all wheels should be the same
            # angles are zero -> wheels point forward
            self.output_command.flsa = 0
            self.output_command.frsa = 0
            self.output_command.blsa = 0
            self.output_command.brsa = 0

            # translate linear velocity to rotational velocity of wheel
            # along with the correct direction
            self.output_command.flv = vBody / self.R * sign_v
            self.output_command.frv = self.output_command.flv
            self.output_command.mlv = self.output_command.flv
            self.output_command.mrv = self.output_command.flv
            self.output_command.blv = self.output_command.flv
            self.output_command.brv = self.output_command.flv

        else:  # moving forward at an angle
            # impose a limit on this
            # get radius for circular motion
            rho = vBody / wBody

            # Make sure wheels can accomodate angle
            if rho < self.rhoMin:
                # if they cannot accomdate, do maximum angle
                rho = self.rhoMin
                # the angular velocity must be changed
                wBody = vBody / rho

            # distance from ICR to side changes depending on side of ICR
            radius_left = rho + sign_w * self.B
            radius_right = rho - sign_w * self.B
            dist_mid_left = rho + sign_w * (self.B + self.mid_wh_offset)
            dist_mid_right = rho - sign_w * (self.B + self.mid_wh_offset)

            # Simple trig to get angle to each wheel
            angle_left = math.atan(self.D / radius_left)
            angle_right = math.atan(self.D / radius_right)

            self.output_command.flsa = (angle_left + angle_right) / 2.0 - math.radians(5)  # todo change constant
            self.output_command.frsa = (angle_left + angle_right) / 2.0

            # incorporate the correct direction of the angular
            # displacement of the wheels
            # multiplying this by the sign of the velocity makes the angular
            # velocity of the rover different than the input, but is of a more
            # natural movement
            self.output_command.flsa *= sign_w
            self.output_command.frsa *= sign_w
            self.output_command.blsa = -self.output_command.flsa
            self.output_command.brsa = -self.output_command.frsa

            # distance to front wheels on each side of rover from ICR
            radius_left_front = math.sqrt(radius_left ** 2 + self.D ** 2)  # distance to port side front wheels
            radius_right_front = math.sqrt(radius_right ** 2 + self.D ** 2)  # starboard side
            # the linear velocity of the front/rear wheels on each side
            vpLin = sign_v * wBody * radius_left_front
            vsLin = sign_v * wBody * radius_right_front

            # the individual velocities of each of the wheels
            self.output_command.flv = vpLin / self.R
            self.output_command.frv = vsLin / self.R
            # notice the middle wheels have different distance to ICR center of rotation
            #self.output_command.mlv = sign_v * dist_mid_left * wBody / self.R
            #self.output_command.mrv = sign_v * dist_mid_right * wBody / self.R

            self.output_command.mlv = self.output_command.flv
            self.output_command.mrv = self.output_command.frv

            self.output_command.blv = self.output_command.flv
            self.output_command.brv = self.output_command.frv

    def pointTurn(self, wBody):
        """
        This method positions the wheels such that the robot turns about its center, without causing any linear movement

        The robot turns on the spot
        :param wBody: Body angular velocity
        """
        wBody = float(wBody)
        # movement may occur to position wheels even if
        # robot is not moving around
        # wheels have specific angle - all of them should form a circle together
        self.output_command.flsa = self.pointSteeringAngle  # forms circle
        self.output_command.frsa = -self.output_command.flsa
        self.output_command.blsa = -self.output_command.flsa
        self.output_command.brsa = self.output_command.flsa

        if abs(wBody) < self.zero:
            # if no velocity, return angles and nothing else
            self.output_command.flv = 0
            self.output_command.frv = 0
            self.output_command.mlv = 0
            self.output_command.mrv = 0
            self.output_command.blv = 0
            self.output_command.brv = 0
        else:
            # configure speeds
            r = self.pointSteeringRadius
            v = wBody * r  # linear velocity of each wheel

            self.output_command.flv = v / self.R  # match angular velocity to rotation of wheel
            self.output_command.frv = -self.output_command.flv  # should all move in circle
            self.output_command.mlv = wBody * self.B / self.R  # same angular velocity
            self.output_command.mrv = -self.output_command.mlv
            self.output_command.blv = self.output_command.flv
            self.output_command.brv = -self.output_command.blv


    def translationalMotion(self, y, x):
        """
        In four wheel configuration, this function allow the robot to move in x and y without introducing any angular
        motion. That is the robot will move front/back, left/right without turning.

        Note that translational motion will always make wheel move in forward direction

        :param y: Front/back movement
        :param x: Left/right movement
        """
        ###########################
        # eventually this function should pick the angle based on the
        # current and accumulated angles of the wheels
        ###########################
        if abs(x) < self.zero and abs(y) < self.zero:
            # no velocity
            self.stop()
        elif abs(x) < self.zero:
            # just forward/zero motion, so can use middle wheels as well
            self.steer(y, 0)

        # determines which side should get the diagonal
        sign_x = sign(x)
        # equivalent direction of wheels
        try:
            theta = math.pi / 2 - math.atan(y / x)
        except ZeroDivisionError:
            theta = 0

        # now find actual direction of wheels, such that
        # the wheels will maintain a similar direction on each side,
        # so that when moving sideways the wheels don't do a full
        # rotation whenever the joystick goes past 90 from forward
        if sign_x > 0 and theta < 0:  # make theta positive
            theta = math.pi - theta
        elif sign_x < 0 and theta > 0:  # make theta negative
            theta = theta - math.pi

        self.output_command.flsa = theta
        self.output_command.frsa = theta
        self.output_command.blsa = theta
        self.output_command.brsa = theta
        # translate linear velocity to rotational velocity of wheel
        # along with the correct direction

        self.output_command.flv = max([abs(x), abs(y)]) / self.R
        self.output_command.frv = self.output_command.flv
        self.output_command.mlv = 0
        self.output_command.mrv = 0
        self.output_command.blv = -self.output_command.flv
        self.output_command.brv = self.output_command.flv
