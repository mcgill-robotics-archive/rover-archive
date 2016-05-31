//
// Created by David Lavoie-Boutin on 2016-05-27.
//

#ifndef ROVER_ARDUINO_BAROMETER_H
#define ROVER_ARDUINO_BAROMETER_H

#include <Arduino.h>
#include <Adafruit_MPL3115A2/Adafruit_MPL3115A2.h>


class Barometer {
public:
    Barometer();

    float get_pressure ();
    float get_altitude ();
    float get_ambient_temperature ();

private:
    Adafruit_MPL3115A2 mBarometer;
};


#endif //ROVER_ARDUINO_BAROMETER_H
