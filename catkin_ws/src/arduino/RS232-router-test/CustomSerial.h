//
// Created by david on 11/10/15.
//

#ifndef ROVER_ARDUINO_CUSTOMSERIAL_H
#define ROVER_ARDUINO_CUSTOMSERIAL_H

#include <Arduino.h>

#define bit9600Delay 100
#define halfBit9600Delay 50

#define rx 6
#define tx 7

void SWprint(int data);
byte SWread();

#endif //ROVER_ARDUINO_CUSTOMSERIAL_H
