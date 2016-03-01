//
// Created by David Lavoie-Boutin on 2016-02-08.
//

#include "Motor.h"

using namespace arm;


Motor::Motor(uint8_t motorPin, uint8_t brakePin, uint8_t directionPin) {
    mMotorPin = motorPin;
    mBrakePin = brakePin;
    mDirectionPin = directionPin;

    pinMode(mMotorPin, OUTPUT);
    pinMode(mBrakePin, OUTPUT);
    pinMode(mDirectionPin, OUTPUT);

    digitalWrite(mBrakePin, 0);
    digitalWrite(mDirectionPin, 0);
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
        digitalWrite(mDirectionPin, (uint8_t) (speed < 0 == mReverseDirection));
        analogWrite(mMotorPin, (int) speed);
    }
}

void Motor::unlock() {
    digitalWrite(mBrakePin, 1);
}

bool Motor::isLocked() {
    return (bool) digitalRead(mBrakePin);
}

bool Motor::isReverseDirection() {
    return mReverseDirection;
}

void Motor::setReverseDirection(bool reverseDirection) {
    mReverseDirection = reverseDirection;
}
