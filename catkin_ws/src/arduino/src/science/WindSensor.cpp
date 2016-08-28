//
// Created by Malcolm Watt and Jeslan Rejendram on 08/15/2016
//

#include "WindSensor.h"

float WindSensor::get_wind_speed(){
    wind_sensor_value = analogRead(pin_wind);
	// calculate sensor voltage from wind sensor
    wind_sensor_voltage = wind_sensor_value * VOLTAGE_CONSTANT;
    if (wind_sensor_voltage <= VOLTAGE_MIN){
      return 0.0;
    }
    else {
	  // calculate actual voltage by subtracting minimum voltage offset
      float measured_voltage = wind_sensor_voltage - VOLTAGE_MIN;
	  // calculate voltage per unit using the maximum and minimum voltage
      float maximum_voltage_range = VOLTAGE_MAX - VOLTAGE_MIN;
	  // calculate unit value of voltage 
      float measured_ratio_of_max = measured_voltage / maximum_voltage_range;
	  // calculate wind speed value from maximum wind speed value and unit value
      return measured_ratio_of_max * WIND_SPEED_MAX;
    }
}
