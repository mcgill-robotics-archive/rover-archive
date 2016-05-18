//
// Created by David Lavoie-Boutin on 2016-02-08.
//

#include <ros.h>
#include "Pololu.h"

using namespace motor;


Pololu::Pololu(uint8_t motorPin, uint8_t brakePin, uint8_t INA, uint8_t INB, ros::NodeHandle *nodeHandle) {
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

Pololu::~Pololu() {

}

void Pololu::lock() {
    digitalWrite(mBrakePin, 0);
    analogWrite(mMotorPin, 0);
}

void Pololu::setSpeed(float speed) {
    if (speed == 0) lock();
    else {
        unlock();
        setDirection(speed < 0);
        analogWrite(mMotorPin, abs(speed));
    }
}

void Pololu::unlock() {
    if (!isLocked()) {
        digitalWrite(mBrakePin, 1);
        delay(30);
    }
}

bool Pololu::isLocked() {
    return (bool) digitalRead(mBrakePin);
}

bool Pololu::isReverseDirection() {
    return (bool) digitalRead(mINA);
}

void Pololu::setDirection(float speed) {
    digitalWrite(mINA, (uint8_t) speed);
    digitalWrite(mINB, (uint8_t) !speed);
}

void Pololu::brake(bool brk) {
    brk ? lock() : unlock();
}

void Pololu::enable(bool en) {
    brake(en);
}

bool Pololu::getStatus() {
    // todo: electrical info for feedback from pololu
    return true;
}