//
// Created by David Lavoie-Boutin on 2016-05-27.
//

#include <Arduino.h>
#include "Sensors.h"


//humidity
int get_humidity ()
{
    return (analogRead(pin_humidity)); //it's just an uncallibrated read
}

//thermocouple
float get_ground_temperature ()
{
    return (((analogRead(pin_thermocouple)*5.0/1023.0)-1.21)/0.005);
}

//void set_servo_switch(bool flag) //sets the direction for servo rotation
//{
//    servoSpin = flag;
//}
