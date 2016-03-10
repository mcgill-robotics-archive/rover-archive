//
// Created by David Lavoie-Boutin on 2016-01-30.
//

#include "PitchRollCompute.h"

using namespace arm;

PitchRollCompute::PitchRollCompute(Encoder *encoder, Encoder *encoder1) {
    mEncoder1 = encoder;
    mEncoder2 = encoder1;

    mPitch = 0;
    mRoll = 0;
}

PitchRollCompute::~PitchRollCompute() { }

void PitchRollCompute::compute(float *pitchRoll, float *position) {
    float val1 = position[0] = mEncoder1->readPosition();
    float val2 = position[1] = mEncoder2->readPosition();

    float largest = val1 > val2 ? val1 : val2;
    float lowest = val1 <= val2 ? val1 : val2;

    mPitch = largest - (largest - lowest) / 2.0;
    mRoll = (largest - lowest) / 2.0;

    mRoll = (largest == val1) ? mRoll : - mRoll;

    pitchRoll[0] = mPitch;
    pitchRoll[1] = mRoll;
}

void PitchRollCompute::inverse(float pitch, float roll, float *destination) {
    destination[0] = pitch + (roll / 2.0);
    destination[1] = - (pitch - (roll / 2.0));
}


