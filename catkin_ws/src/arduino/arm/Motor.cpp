//
// Created by David Lavoie-Boutin on 2016-02-08.
//

#include "Motor.h"

using namespace arm;


Motor::Motor(uint8_t motorPin, uint8_t brakePin, uint8_t INA, uint8_t INB) {
    mMotorPin = motorPin;
    mBrakePin = brakePin;
    mINA = INA;
    mINB = INB;

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
    return (bool) digitalRead(mINA);
}

void Motor::setReverseDirection(bool reverseDirection) {
    digitalWrite(mINA, (uint8_t) reverseDirection);
    digitalWrite(mINA, (uint8_t) !reverseDirection);
}
