//
// Created by David Lavoie-Boutin on 2016-02-08.
//

#ifndef ROVER_ARDUINO_RAM_H
#define ROVER_ARDUINO_RAM_H

#include <arduino/ram.h>

int freeRam ();
void freeRamCallback(const arduino::ram::Request & request, arduino::ram::Response & response);

#endif //ROVER_ARDUINO_RAM_H
