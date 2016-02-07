//
// Created by David Lavoie-Boutin on 2016-01-30.
//

#include <Arduino.h>
#include <SPI/SPI.h>
#include "Encoder.h"

using namespace arm;

Encoder::Encoder(uint8_t pin, ros::NodeHandle *nh) {
    mNh = nh;
    mPin = pin;
    mOffset = 0;
}

Encoder::~Encoder() {

}

float Encoder::readPosition() {
    digitalWrite(mPin, LOW);

    byte dA = SPI.transfer(0x00);
    byte dB = SPI.transfer(0x00);
    int x= ((dA & 0x7F)<<6) | (dB>>2);
    float ax = x*359.956/8191.000;

    digitalWrite(mPin, HIGH);
    return ax + mOffset;
}

void Encoder::setOffset(float offset) {
    mOffset = offset;
}

float Encoder::getOffset() {
    return mOffset;
}
