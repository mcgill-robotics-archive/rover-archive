//
// Created by David Lavoie-Boutin on 2016-01-30.
//

#include <Arduino.h>
#include <SPI/SPI.h>
#include "Encoder.h"

using namespace arm;

Encoder::Encoder(uint8_t pin, bool inverted, ros::NodeHandle *nh) {
    mNh = nh;
    mPin = pin;
    mOffset = 0;
    dA = 0;
    dB = 0;
    x = 0;
    ax = 0;
    mInverted = inverted;
    pinMode(mPin, OUTPUT);
}

Encoder::~Encoder() {

}

float Encoder::readPosition() {
    digitalWrite(mPin, LOW);

    dA = SPI.transfer(0x00);
    dB = SPI.transfer(0x00);

    x= ((dA & 0x7F)<<6) | (dB>>2);
    ax = x * 359.956/8191.000;

    digitalWrite(mPin, HIGH);

    if (mInverted){
        ax = 360.0 - ax;
    }

    return ax + mOffset;
}

void Encoder::setOffset(float offset) {
    mOffset = offset;
}

float Encoder::getOffset() {
    return mOffset;
}
