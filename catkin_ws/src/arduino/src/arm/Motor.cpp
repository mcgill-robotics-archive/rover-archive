//
// Created by David Lavoie-Boutin on 2016-02-08.
//

#include <ros.h>
#include "Motor.h"

using namespace arm;


Motor::Motor(uint8_t motorPin, uint8_t brakePin, uint8_t INA, uint8_t INB, ros::NodeHandle *nodeHandle) {
    mMotorPin = motorPin;
    mBrakePin = brakePin;
    mINA = INA;
    mINB = INB;
    mNh = nodeHandle;

    pinMode(mMotorPin, OUTPUT);
    pinMode(mBrakePin, OUTPUT);
    pinMode(mINA, OUTPUT);
    pinMode(mINB, OUTPUT);

    digitalWrite(mBrakePin, 0);
    digitalWrite(mINA, 0);
    digitalWrite(mINB, 1);
    analogWrite(mMotorPin, 0);
}

Motor::~Motor() {

}

void Motor::lock() {
    digitalWrite(mBrakePin, 0);
    analogWrite(mMotorPin, 0);
}

void Motor::setSpeed(double speed) {
    if (speed == 0) lock();
    else {
        unlock();
        setReverseDirection(speed < 0);
        analogWrite(mMotorPin, (int) abs(speed));
    }
}

void Motor::unlock() {
    if (!isLocked()) {
        digitalWrite(mBrakePin, 1);
        delay(30);
    }
}

bool Motor::isLocked() {
    return (bool) digitalRead(mBrakePin);
}

bool Motor::isReverseDirection() {
    return (bool) digitalRead(mINA);
}

void Motor::setReverseDirection(bool reverseDirection) {
    digitalWrite(mINA, (uint8_t) reverseDirection);
    digitalWrite(mINB, (uint8_t) !reverseDirection);
}
