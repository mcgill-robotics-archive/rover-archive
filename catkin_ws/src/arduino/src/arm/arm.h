//
// Created by David Lavoie-Boutin on 2016-02-07.
//

#ifndef ROVER_ARDUINO_ARM_H
#define ROVER_ARDUINO_ARM_H

#include <ros.h>

// Select position or speed control
bool pid = false;

// Current position read from encoders
float * pitchRollLink1 = new float[2]; // first index is pitch, second is roll
float * pitchRollLink2 = new float[2]; // first index is pitch, second is roll
float * diff1pos = new float[2]; // first index is left, second is right
float * diff2pos = new float[2]; // first index is left, second is right
float baseYawPosition = 0;
float pitch1Position = 0;
float endEffectorPosition = 0;

// Target position for position control
double baseYawSetPoint = 0;
double pitch1SetPoint = 0;
float * diff1setPoint = new float[2]; // first index is left, second is right
float * diff2setPoint = new float[2]; // first index is left, second is right

// Motor speeds for position control
double baseYawOutput = 0;
double pitch1Output = 0;
double diff1leftOutput = 0;
double diff2leftOutput = 0;
double diff1rightOutput = 0;
double diff2rightOutput = 0;
double endEffectorOutput = 0;

// Motor speeds for velocity control
double baseYawOutputVel = 0;
double pitch1OutputVel = 0;
double endEffectorOutputVel = 0;
float * diff1Vel = new float[2]; // first index is left, second is right
float * diff2Vel = new float[2]; // first index is left, second is right

#endif //ROVER_ARDUINO_ARM_H
