//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#include "MAXON.h"

using namespace drive;

MAXON::MAXON(uint8_t speedPin, uint8_t directionPin, uint8_t enablePin, uint8_t data1Pin, uint8_t data2Pin,
                    uint8_t feedbackPin, MotorConfig::Mode mode) {
    mSpeedPin = speedPin;
    mData1Pin = data1Pin;
    mData2Pin = data2Pin;
    mDirectionPin = directionPin;
    mEnablePin = enablePin;
    mFeedbackPin = feedbackPin;

    pinMode(speedPin, OUTPUT);
    pinMode(data1Pin, OUTPUT);
    pinMode(data2Pin, OUTPUT);
    pinMode(directionPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(feedbackPin, INPUT);

    analogWrite(speedPin, 0);
    digitalWrite(data1Pin, 0);
    digitalWrite(data2Pin, 0);
    digitalWrite(directionPin, 0);
    digitalWrite(enablePin, 0);

    setMode(mode);
}

void MAXON::setSpeed(int speed) {
    speed = constrain(speed, 0, 255);
    analogWrite(mSpeedPin, speed);
}

void MAXON::setDirection(int dir) {
    if (dir < 0) digitalWrite(mDirectionPin, 0);
    else if (dir > 0) digitalWrite(mDirectionPin, 1);
}

void MAXON::brake(bool brk) {
    if (brk)
    {
        analogWrite(mSpeedPin, 0);
        enable(false);

    }
}
void MAXON::enable(bool en) {
    digitalWrite(mEnablePin, (uint8_t) en);
}

bool MAXON::getStatus() {
    return (bool) digitalRead(mFeedbackPin);
}

void MAXON::setMode(MotorConfig::Mode mode) {
    switch (mode) {
        case MotorConfig::Mode::OpenLoop :
            digitalWrite(mData1Pin, LOW);
            digitalWrite(mData2Pin, LOW);
            break;
        case MotorConfig::Mode::Slow:
            digitalWrite(mData1Pin, HIGH);
            digitalWrite(mData2Pin, LOW);
            break;
        case MotorConfig::Mode::Medium:
            digitalWrite(mData1Pin, LOW);
            digitalWrite(mData2Pin, HIGH);
            break;
        case MotorConfig::Mode::Fast:
            digitalWrite(mData1Pin, HIGH);
            digitalWrite(mData2Pin, HIGH);
            break;
        default:
            break;
    }
}
