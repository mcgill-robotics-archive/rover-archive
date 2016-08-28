//
// Created by Malcolm Watt and Jeslan Rejendram on 08/15/2016
//

#ifndef ROVER_ARDUINO_WINDSENSOR_H
#define ROVER_ARDUINO_WINDSENSOR_H

#include <Arduino.h>

#define pin_wind A2


class WindSensor{
    // Constants for the wind readings
    int wind_sensor_value = 0;
    float wind_sensor_voltage = 0.0;
    float VOLTAGE_CONSTANT = 0.004882814;
    float VOLTAGE_MIN = 0.4;
    float VOLTAGE_MAX = 2.0;
    float WIND_SPEED_MAX = 32.0;
public:
    /*
     * Returns the wind speed in m/s.
     */
    float get_wind_speed();
};

#endif //ROVER_ARDUINO_WINDSENSOR_H
