//
// Created by David Lavoie-Boutin on 2016-01-30.
//

#ifndef ROVER_ARDUINO_ENCODER_H
#define ROVER_ARDUINO_ENCODER_H


#include <stdint.h>
#include <ros.h>

namespace arm {

/**
 * \brief Interface to the arm encoders.
 *
 * We use the SPI bus to communicate with the encoders.
 *
 * This class only drives the slave select line and transfers data on the SPI bus.
 *
 * <b>Note: </b> The SPI bus configuration and initialisation should be done be the caller of <code> readPosition() </code>
 */
class Encoder {
public:
    /**
     * \brief Public constructor, sets the direction of the digital pin
     *
     * \param pin Pin number for the encoder selection. Drives the SS pin of the SPI bus
     * \param nh Pointer to a valid ros handle for logging
     */
    Encoder(uint8_t pin, bool inverted, ros::NodeHandle *nh);
    virtual ~Encoder();

    /**
     * \brief Get the absolute position of the encoder
     *
     * \return Current position of the encoder on the 0 to 360 degrees circle
     */
    float readPosition(); // ensure spi initialisation has been done prior to ordering the read

    /**
     * \brief Set possible offset to account for imprecise mounting
     *
     * \param offset The value to be added at each read computation
     */
    void setOffset(float offset);

    /**
     * \brief Get the current offset value
     *
     * \return The current offset value
     */
    float getOffset();

private:
    uint8_t mPin;
    float mOffset;
    ros::NodeHandle * mNh;

    byte dA, dB;
    int x;
    float ax;
    bool mInverted;
};
}


#endif //ROVER_ARDUINO_ENCODER_H
