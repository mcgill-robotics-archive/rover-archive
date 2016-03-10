//
// Created by David Lavoie-Boutin on 2016-02-07.
//

#ifndef ROVER_ARDUINO_ARM_H
#define ROVER_ARDUINO_ARM_H

#include <ros.h>

char array[30];
int x;
byte dA;
byte dB;

float * pitchRollLink1 = new float[2]; // first index is pitch, second is roll
float * pitchRollLink2 = new float[2]; // first index is pitch, second is roll
float * diff1pos = new float[2]; // first index is left, second is right
float * diff2pos = new float[2]; // first index is left, second is right
float * diff1setPoint = new float[2]; // first index is left, second is right
float * diff2setPoint = new float[2]; // first index is left, second is right

float baseYawPosition = 0;
float pitch1Position = 0;
float endEffectorPosition = 0;

double baseYawSetPoint = 0;
double pitch1SetPoint = 0;
double diff1leftSetPoint = 0;
double diff2leftSetPoint = 0;
double diff1rightSetPoint = 0;
double diff2rightSetPoint = 0;

double baseYawOutput = 0;
double pitch1Output = 0;
double diff1leftOutput = 0;
double diff2leftOutput = 0;
double diff1rightOutput = 0;
double diff2rightOutput = 0;
double endEffectorOutput = 0;

#endif //ROVER_ARDUINO_ARM_H
