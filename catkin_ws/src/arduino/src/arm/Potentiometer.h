//
// Created by david on 5/15/16.
//

#ifndef ROVER_ARDUINO_POTENTIOMETER_H
#define ROVER_ARDUINO_POTENTIOMETER_H

#include <Arduino.h>
#include <ros.h>

class Potentiometer {
public:

    Potentiometer(uint8_t pin, bool inverted, float factor, ros::NodeHandle *nh);

    float readPosition();
    void setOffset(float offset);
    float getOffset();

private:
    uint8_t mPin;
    float mOffset;
    float mFactor;
    bool mInverted;
};


#endif //ROVER_ARDUINO_POTENTIOMETER_H
