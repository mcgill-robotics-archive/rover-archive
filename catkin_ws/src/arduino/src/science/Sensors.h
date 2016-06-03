//
// Created by David Lavoie-Boutin on 2016-05-27.
//

#ifndef ROVER_ARDUINO_SENSORS_H
#define ROVER_ARDUINO_SENSORS_H

//for the ph sensor


//defines for pins
//TODO: fill these in with the actual pinout. the ones here are just placeholders

#define pin_humidity A1
#define pin_thermocouple A0
#define pin_augur_servo_limit_switch 13

int get_humidity ();
float get_ground_temperature ();


#endif //ROVER_ARDUINO_SENSORS_H
