//
// Created by David Lavoie-Boutin on 2016-01-30.
//

#ifndef ROVER_ARDUINO_ENCODER_H
#define ROVER_ARDUINO_ENCODER_H


#include <stdint.h>
namespace arm {

/**
 *
 */
class Encoder {
public:
    Encoder(uint8_t);
    virtual ~Encoder();

    float readPosition(); // ensure spi initialisation has been done prior to ordering the read
    void setOffset(float);
    float getOffset();

private:
    uint8_t mPin;
    float mOffset;
};
}


#endif //ROVER_ARDUINO_ENCODER_H
