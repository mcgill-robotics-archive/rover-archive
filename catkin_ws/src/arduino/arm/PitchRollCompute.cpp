//
// Created by David Lavoie-Boutin on 2016-01-30.
//

#include "PitchRollCompute.h"

using namespace arm;

PitchRollCompute::PitchRollCompute(Encoder *encoder, Encoder *encoder1) {
    mEncoder1 = encoder;
    mEncoder2 = encoder1;

    pitch = 0;
    roll = 0;
}

PitchRollCompute::~PitchRollCompute() { }

void PitchRollCompute::compute(float *feedback) {
    float val1 = mEncoder1->readPosition();
    float val2 = mEncoder2->readPosition();

    float largest = val1 > val2 ? val1 : val2;
    float lowest = val1 <= val2 ? val1 : val2;

    pitch = largest - (largest - lowest) / 2.0;
    roll = (largest - lowest) / 2.0;

    feedback[0] = pitch;
    feedback[1] = roll;
}
