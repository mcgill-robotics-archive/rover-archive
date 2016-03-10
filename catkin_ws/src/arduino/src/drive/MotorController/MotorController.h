//
// Created by David Lavoie-Boutin on 2016-01-18.
//

#ifndef ROVER_ARDUINO_MOTORCONTROLLER_H
#define ROVER_ARDUINO_MOTORCONTROLLER_H

#include "MotorConfig.h"
namespace drive {

class MotorController {
public:
    static MotorController * createMotorController(MotorConfig motorConfig);

    virtual void setSpeed(int) = 0;
    virtual void setDirection(int) = 0;
    virtual void brake(bool) = 0;
    virtual void enable(bool) = 0;
    virtual bool getStatus() = 0;

    virtual ~MotorController();

protected:
    MotorController();


};
}


#endif //ROVER_ARDUINO_MOTORCONTROLLER_H
