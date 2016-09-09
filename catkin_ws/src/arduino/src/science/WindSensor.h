//
// Created by Malcolm Watt and Jeslan Rejendram on 08/15/2016
//

#ifndef ROVER_ARDUINO_WINDSENSOR_H
#define ROVER_ARDUINO_WINDSENSOR_H

#include <Arduino.h>

#define pin_wind A2

class WindSensor{
public:

    WindSensor(uint8_t pin = pin_wind);
    /*
     * Returns the wind speed in m/s.
     */
    float get_wind_speed();

private:
    // Constants for the wind readings
    static const float VOLTAGE_CONSTANT = 0.004882814; /// voltage per division of the arduino ADC
    static const float VOLTAGE_MIN = 0.4;
    static const float VOLTAGE_MAX = 2.0;
    static const float WIND_SPEED_MAX = 32.0;

    float maximum_voltage_range;
    uint8_t pin;

};

#endif //ROVER_ARDUINO_WINDSENSOR_H
