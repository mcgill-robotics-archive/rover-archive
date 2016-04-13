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

void PitchRollCompute::compute(double *pitchRoll, double *position) {
    float val1 = position[0] = mEncoderLeft->readPosition();
    float val2 = position[1] = mEncoderRight->readPosition();

    float largest = val1 > val2 ? val1 : val2;
    float lowest = val1 <= val2 ? val1 : val2;

    mPitch = 180 - largest - (largest - lowest) / 2.0;
    mRoll = (largest - lowest) / 2.0;

    mRoll = (largest == val1) ? mRoll : - mRoll;

    pitchRoll[0] = mPitch;
    pitchRoll[1] = mRoll;
}

void PitchRollCompute::inverse(float pitch, float roll, double *destination) {
    destination[0] = pitch + (roll / 2.0);
    destination[1] = - (pitch - (roll / 2.0));
}


