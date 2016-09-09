//
// Created by Malcolm Watt and Jeslan Rejendram on 08/15/2016
//

#include "WindSensor.h"

float WindSensor::get_wind_speed(){
    float wind_sensor_voltage = analogRead(pin) * VOLTAGE_CONSTANT;

    if (wind_sensor_voltage <= VOLTAGE_MIN){
        return 0.0;
    }

    else {
        return (wind_sensor_voltage - VOLTAGE_MIN)*WIND_SPEED_MAX/(maximum_voltage_range);
    }
}

WindSensor::WindSensor(uint8_t pin) : pin(pin) {
    // calculate voltage per unit using the maximum and minimum voltage
    maximum_voltage_range = VOLTAGE_MAX - VOLTAGE_MIN;
}
