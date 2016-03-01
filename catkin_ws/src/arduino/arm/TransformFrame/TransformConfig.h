//
// Created by David Lavoie-Boutin on 2016-01-31.
//

#ifndef ROVER_ARDUINO_TRANSFORMCONFIG_H
#define ROVER_ARDUINO_TRANSFORMCONFIG_H

namespace arm {

class TransformConfig {
public:
    TransformConfig();

    const char *baseFrame;
    const char *baseYawFrame;
    const char *pitch1Frame;
    const char *pitch2Frame;
    const char *roll1Frame;
    const char *pitch3Frame;
    const char *roll2Frame;

    static const double armPosition = 0.10; //distance between arm and robot frame
    static const double pitch1offset = 0.02;
    static const double pitch2offset = 0.10;
    static const double roll1offset = 0.02;
    static const double pitch3offset = 0.10;
    static const double roll2offset = 0.02;
};
}

#endif //ROVER_ARDUINO_TRANSFORMCONFIG_H
