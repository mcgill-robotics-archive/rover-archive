//
// Created by David Lavoie-Boutin on 2016-01-30.
//

#include "PitchRollCompute.h"

using namespace arm;

PitchRollCompute::PitchRollCompute(Encoder *encoderLeft, Encoder *encoderRight) {
    mEncoderLeft = encoderLeft;
    mEncoderRight = encoderRight;

    mPitch = 0;
    mRoll = 0;
}

PitchRollCompute::~PitchRollCompute() { }

void PitchRollCompute::compute(double *pitch, double *roll, double *positionLeft, double *positionRight) {
    float val1 = *positionLeft = mEncoderLeft->readPosition();
    float val2 = *positionRight = mEncoderRight->readPosition();

    float largest = val1 > val2 ? val1 : val2;
    float lowest = val1 <= val2 ? val1 : val2;

    mPitch = 180 - largest - (largest - lowest) / 2.0;
    mRoll = (largest - lowest) / 2.0;

    mRoll = (largest == val1) ? mRoll : - mRoll;

    *pitch = mPitch;
    *roll = mRoll;
}

void PitchRollCompute::inverseSpeed(float pitch, float roll, double *destination) {
    destination[0] = pitch + (roll / 2.0);
    destination[1] = - (pitch - (roll / 2.0));
}


void PitchRollCompute::inversePosition(float pitch, float roll, double * left, double * right){
    *left = pitch + roll / 2.0;
    *right = pitch - roll / 2.0;
}


