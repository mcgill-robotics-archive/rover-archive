//
// Created by david on 4/25/16.
//

#include "AfroESC.h"

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void motor::AfroESC::brake(bool brk) {
    if (brk) setSpeed(0);
}

void motor::AfroESC::enable(bool en) {
    mEnabled = en;
    if (!en)    setSpeed(0);
}

bool motor::AfroESC::getStatus() {
    return true;
}

void motor::AfroESC::setDirection(float speed) {
    directionSign = speed > 0 ? 1 : -1;
}

void motor::AfroESC::setSpeed(float speed) {
    if (mEnabled) {
        int mapSpeed = (int) map(abs(speed), 0, 100, 0, 400);
        setDirection(speed);
        servo.writeMicroseconds(1500 + directionSign * mapSpeed);
    }
    else servo.writeMicroseconds(1500);
}

motor::AfroESC::~AfroESC() {

}

motor::AfroESC::AfroESC(uint8_t motorPin) : mMotorPin(motorPin) {
    directionSign = 1;
    mEnabled = false;
    pinMode(mMotorPin, OUTPUT);
    servo.attach(mMotorPin);
    servo.writeMicroseconds(1500);
}



