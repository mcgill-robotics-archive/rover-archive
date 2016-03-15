//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#include "include/MotorConfig.h"

motor::MotorConfig::MotorConfig() {
    data1Pin = 0;
    data2Pin = 0;
    feedbackPin = 0;
    brakePin = 0;
    slaveSelectPin = 0;
    faultPin = 0;
    resetPin = 0;
    speedPin = 0;
    directionPin = 0;
    enablePin = 0;
    controllerType = _MAXON;
    mode = OpenLoop;
}
