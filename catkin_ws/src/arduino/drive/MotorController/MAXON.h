//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#ifndef ROVER_ARDUINO_MAXON_H
#define ROVER_ARDUINO_MAXON_H

#include "MotorController.h"
#include "MotorConfig.h"
namespace drive {

class MAXON : public MotorController {
public:
    MAXON(uint8_t speedPin,

    uint8_t directionPin,
    uint8_t enablePin,
    uint8_t data1Pin,
    uint8_t data2Pin,
    uint8_t feedbackPin,
    MotorConfig::Mode mode);

    virtual void setSpeed(int);
    virtual void setDirection(int);
    virtual void brake(bool);
    virtual void enable(bool);
    virtual bool getStatus();

private:
    uint8_t mData1Pin;
    uint8_t mData2Pin;
    uint8_t mFeedbackPin;
    uint8_t mSpeedPin;
    uint8_t mDirectionPin;
    uint8_t mEnablePin;

    void setMode(MotorConfig::Mode mode);
};
}


#endif //ROVER_ARDUINO_MAXON_H
