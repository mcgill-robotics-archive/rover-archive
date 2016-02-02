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

    static const int armPosition = 10; //distance between arm and robot frame
    static const int pitch1offset = 2;
    static const int pitch2offset = 10;
    static const int roll1offset = 2;
    static const int pitch3offset = 10;
    static const int roll2offset = 2;
};
}

#endif //ROVER_ARDUINO_TRANSFORMCONFIG_H
