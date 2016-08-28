//
// Created by David Lavoie-Boutin on 2016-05-27.
//

#include <Arduino.h>
#include "Humidity.h"

// We need to add calibration, as well as the nodeHandle for the humidity sensor
int get_humidity ()
{
    return (analogRead(pin_humidity));
}
