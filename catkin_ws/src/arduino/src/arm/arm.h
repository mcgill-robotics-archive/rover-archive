//
// Created by David Lavoie-Boutin on 2016-02-07.
//

#ifndef ROVER_ARDUINO_ARM_H
#define ROVER_ARDUINO_ARM_H

// Select position or speed control
bool pid = false;

// Target position for position control
double * baseYawSetPoint = new double;
double * pitch1SetPoint = new double;
double * diff1setPointLeft = new double;
double * diff1setPointRight = new double;
double * diff2setPointLeft = new double;
double * diff2setPointRight = new double;

double endEffectorOutput = 0;

// Motor speeds for velocity control
double baseYawOutputVel = 0;
double pitch1OutputVel = 0;
double endEffectorOutputVel = 0;
double * diff1Vel = new double[2]; // first index is left, second is right
double * diff2Vel = new double[2]; // first index is left, second is right

#endif //ROVER_ARDUINO_ARM_H
