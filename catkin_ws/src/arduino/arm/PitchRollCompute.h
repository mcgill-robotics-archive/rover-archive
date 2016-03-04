//
// Created by David Lavoie-Boutin on 2016-01-30.
//

#ifndef ROVER_ARDUINO_PITCHROLLCOMPUTE_H
#define ROVER_ARDUINO_PITCHROLLCOMPUTE_H

#include "Encoder.h"

namespace arm{

class PitchRollCompute {

public:
    PitchRollCompute(Encoder*, Encoder*);
    ~PitchRollCompute();
    void compute(float *);
    void inverse(float pitch, float roll, float *destination);

    float mPitch;
    float mRoll;

private:
    Encoder* mEncoder1;
    Encoder* mEncoder2;
};
}


#endif //ROVER_ARDUINO_PITCHROLLCOMPUTE_H
