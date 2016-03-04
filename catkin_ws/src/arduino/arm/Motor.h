//
// Created by David Lavoie-Boutin on 2016-02-08.
//

#ifndef ROVER_ARDUINO_MOTOR_H
#define ROVER_ARDUINO_MOTOR_H

#include <Arduino.h>
#include <ros.h>

namespace arm {

class Motor {
public:
    Motor(uint8_t motorPin, uint8_t brakePin, uint8_t INA, uint8_t INB, ros::NodeHandle *nodeHandle);
    ~Motor();

    void lock();
    void setSpeed(double speed);
    void unlock();
    bool isLocked();

    bool isReverseDirection();
    void setReverseDirection(bool reverseDirection);

private:
    uint8_t mMotorPin;
    uint8_t mBrakePin;
    uint8_t mINA;
    uint8_t mINB;
    ros::NodeHandle * mNh;
};

}

#endif //ROVER_ARDUINO_MOTOR_H
