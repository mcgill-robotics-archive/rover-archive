//
// Created by david on 4/25/16.
//

#ifndef ROVER_ARDUINO_AFROESC_H
#define ROVER_ARDUINO_AFROESC_H

#include <include/MotorController.h>
#include <Servo.h>

namespace motor {

class AfroESC : public MotorController {


public:
    AfroESC(uint8_t motorPin);

    virtual ~AfroESC();

    virtual void setSpeed(float speed);

    virtual void setDirection(float speed);

    virtual void brake(bool brk);

    virtual void enable(bool en);

    virtual bool getStatus();

private:
    uint8_t mMotorPin;
    Servo servo;
    int directionSign;
    bool mEnabled;
};

}

#endif //ROVER_ARDUINO_AFROESC_H
