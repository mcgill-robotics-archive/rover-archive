//
// Created by David Lavoie-Boutin on 2016-05-27.
//

#include "Barometer.h"


float Barometer::get_pressure() {

    return ((mBarometer.getPressure()/1000) + 100);
}

float Barometer::get_ambient_temperature() {
    return mBarometer.getTemperature();
}

float Barometer::get_altitude() {
    return mBarometer.getAltitude();
}

Barometer::Barometer() {
    mBarometer.begin();
}
