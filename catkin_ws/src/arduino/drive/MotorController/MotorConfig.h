//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#ifndef ROVER_ARDUINO_MOTORCONTROLLERCONFIG_H
#define ROVER_ARDUINO_MOTORCONTROLLERCONFIG_H

#include <Arduino.h>
namespace drive {

enum ControllerType {
    _MAXON = 0,
    _DRV8308 = 1
};

enum Mode{
    OpenLoop = 0,
    Slow = 1,
    Medium = 2,
    Fast = 3
};

class MotorConfig {
public:
    ControllerType controllerType;
    Mode mode;

    uint8_t data1Pin;
    uint8_t data2Pin;
    uint8_t feedbackPin;

    uint8_t brakePin;
    uint8_t slaveSelectPin;
    uint8_t faultPin;
    uint8_t resetPin;

    uint8_t speedPin;
    uint8_t directionPin;
    uint8_t enablePin;


    MotorConfig();
    virtual ~MotorConfig() {};
};

}

#endif //ROVER_ARDUINO_MOTORCONTROLLERCONFIG_H
