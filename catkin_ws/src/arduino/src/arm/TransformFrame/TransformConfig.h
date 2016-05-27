//
// Created by David Lavoie-Boutin on 2016-01-31.
//

#ifndef ROVER_ARDUINO_TRANSFORMCONFIG_H
#define ROVER_ARDUINO_TRANSFORMCONFIG_H

namespace arm {

/**
 * \brief Configuration object encapsulating all the constants relevant to arm transforms
 */
class TransformConfig {
public:
    TransformConfig();

    /// Name of base tf frame for the arm
    const char *baseFrame;
    /// Name of frame representing base yaw
    const char *baseYawFrame;
    /// Name of frame for base pitch angle
    const char *pitch1Frame;
    /// Name of frame for first differential pitch angle
    const char *pitch2Frame;
    /// Name of frame for first differential roll angle
    const char *roll1Frame;
    /// Name of frame for second differential pitch angle
    const char *pitch3Frame;
    /// Name of frame for second differential roll angle
    const char *roll2Frame;

    /// Distance between arm and robot frame
    static const double armPosition = 0.10;
    /// Elevation of the first pitch joint above roll joint
    static const double pitch1offset = 0.02;
    /// Length of the first link
    static const double pitch2offset = 0.10;
    /// Distance between pitch and roll axies
    static const double roll1offset = 0.02;
    /// Length of the second link
    static const double pitch3offset = 0.10;
    /// Distance between pitch and roll axies
    static const double roll2offset = 0.02;
};
}

#endif //ROVER_ARDUINO_TRANSFORMCONFIG_H
