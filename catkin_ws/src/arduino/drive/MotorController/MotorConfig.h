//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#ifndef ROVER_ARDUINO_MOTORCONTROLLERCONFIG_H
#define ROVER_ARDUINO_MOTORCONTROLLERCONFIG_H

#include <Arduino.h>
namespace drive {

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

    enum ControllerType {
        MAXON,
        DRV8308
    };

    enum Mode{
        OpenLoop,
        Slow,
        Medium,
        Fast
    };

    MotorConfig();
    virtual ~MotorConfig() {};
};

}

#endif //ROVER_ARDUINO_MOTORCONTROLLERCONFIG_H
