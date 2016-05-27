//
// Created by david on 5/15/16.
//

#include "Potentiometer.h"

Potentiometer::Potentiometer(uint8_t pin, bool inverted, float factor, ros::NodeHandle *nh) {
    mPin = pin;
    mInverted = inverted;
    mFactor = factor;
    pinMode(mPin, OUTPUT);

}

float Potentiometer::readPosition() {
    int read = analogRead(mPin);
    if (mInverted)
        return 360 - (read * mFactor + mOffset);
    return read * mFactor + mOffset;
}

void Potentiometer::setOffset(float offset) {
    mOffset = offset;
}

float Potentiometer::getOffset() {
    return mOffset;
}







