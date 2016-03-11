//
// Created by David Lavoie-Boutin on 2016-01-30.
//

#ifndef ROVER_ARDUINO_PITCHROLLCOMPUTE_H
#define ROVER_ARDUINO_PITCHROLLCOMPUTE_H

#include "Encoder.h"

namespace arm{

/**
 * \brief Class to encapsulate the conversion from differential gear position
 * to pitch and roll and inverse.
 */
class PitchRollCompute {

public:
    /**
     * \brief Public constructor
     *
     * \param encoderLeft Pointer to the left encoder of the differential pair
     * \param encoderRight Pointer to the right encoder of the differential pair
     */
    PitchRollCompute(Encoder *encoderLeft, Encoder *encoderRight);
    ~PitchRollCompute();

    /**
     * \brief Read encoders and compute current pitch and roll of the joint.
     *
     *
     * \param pitchRoll Pointer to a destination array for the computed values of
     * pitch and roll. Index 0 is pitch, index 1 is roll.
     * \param position Pointer to a destination array where we place the current
     * position of the encoders. Index 0 is left, index 1 is right.
     */
    void compute(float *pitchRoll, float *position);

    /**
     * \brief Compute the position each motor should be at to achieve global pitch adn roll
     *
     * \param pitch Desired pitch
     * \param roll Desired roll
     * \param destination Pointer to array to place target motor output
     */
    void inverse(float pitch, float roll, float *destination);


private:

    float mPitch;
    float mRoll;
    Encoder* mEncoderLeft;
    Encoder* mEncoderRight;
};
}


#endif //ROVER_ARDUINO_PITCHROLLCOMPUTE_H
