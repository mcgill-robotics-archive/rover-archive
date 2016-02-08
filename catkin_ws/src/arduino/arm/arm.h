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
float baseYawValue = 0;
float pitch1Value = 0;

double baseYawSetPoint = 0;
double pitch1SetPoint = 0;
double pitch2SetPoint = 0;
double pitch3SetPoint = 0;
double roll1SetPoint = 0;
double roll2SetPoint = 0;

double baseYawOutput = 0;
double pitch1Output = 0;
double pitch2Output = 0;
double pitch3Output = 0;
double roll1Output = 0;
double roll2Output = 0;

#endif //ROVER_ARDUINO_ARM_H
