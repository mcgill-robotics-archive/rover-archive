//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#ifndef ROVER_ARDUINO_MOTORCONTROLLERCONFIG_H
#define ROVER_ARDUINO_MOTORCONTROLLERCONFIG_H

#include <Arduino.h>
namespace motor {

/**
 * \brief The two motor controllers currently supported
 */
enum ControllerType {
    _MAXON = 0,
    _DRV8308 = 1,
    _POLOLU = 2
};

/**
 * \brief The Maxon motor controller support different operation modes
 */
enum Mode{
    OpenLoop = 0,
    Slow = 1,
    Medium = 2,
    Fast = 3
};

/**
 * \brief Configuration object used by the factory to create the proper motor controller
 */
class MotorConfig {
public:
    /// Physical controller choice
    ControllerType controllerType;
    /// Operation mode for the maxon controller
    Mode mode;

    /// D1 digital pin <b>MAXON ONLY</b>
    uint8_t data1Pin;
    /// D2 digital pin <b>MAXON ONLY</b>
    uint8_t data2Pin;
    /// Motor status feedback digital pin
    uint8_t feedbackPin;

    /// Digital pin for brake <b>DRV8308 ONLY</b>
    uint8_t brakePin;
    /// Slave select pin for SPI bus. Used to poll proper encoder counter.
    uint8_t slaveSelectPin;
    /// Fault feedback pin <b>DRV8308 ONLY</b>
    uint8_t faultPin;
    /// Reset pin <b>DRV8308 ONLY</b>
    uint8_t resetPin;

    /// PWM output pin driving the speed
    uint8_t speedPin;
    /// Digital output for direction
    uint8_t directionPin;
    /// Digital pin to enable or disable the motor controller
    uint8_t enablePin;

    /// Constructor defaults all to 0
    MotorConfig();
    virtual ~MotorConfig() {};
};

}

#endif //ROVER_ARDUINO_MOTORCONTROLLERCONFIG_H
