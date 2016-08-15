//
// Created by Malcolm Watt and Jeslan Rejendram on 08/15/2016
//

#include "WindSensor.h"

float WindSensor::get_wind_speed(){
    wind_sensor_value = analogRead(pin_wind);
    wind_sensor_voltage = wind_sensor_value * VOLTAGE_CONSTANT;
    if (wind_sensor_voltage <= VOLTAGE_MIN){
      return 0.0;
    }
    else {
      float measured_voltage = wind_sensor_voltage - VOLTAGE_MIN;
      float maximum_voltage_range = VOLTAGE_MAX - VOLTAGE_MIN;

      float measured_ratio_of_max = measured_voltage / maximum_voltage_range;
      return measured_ratio_of_max * WIND_SPEED_MAX;
    }
}
