//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#ifndef ROVER_ARDUINO_DRV8308_H
#define ROVER_ARDUINO_DRV8308_H

#include "MotorController.h"

namespace  drive {
class DRV8308 : public MotorController{
public:
    DRV8308() { };
    virtual ~DRV8308() { };

    virtual void setSpeed(int);
    virtual void setDirection(int);
    virtual void brake(bool);
    virtual void enable(bool);
    virtual bool getStatus();

};
}


#endif //ROVER_ARDUINO_DRV8308_H
