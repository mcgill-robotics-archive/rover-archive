//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#ifndef ROVER_ARDUINO_WHEEL_H
#define ROVER_ARDUINO_WHEEL_H

#include "Arduino.h"
#include <ros.h>

namespace drive {

class Wheel {

public:
    /**
     * Public constructor.
     * \param motorPort is the pin number to the motor controller
     * \param nodeHandle pointer to the node handle to use for logging.
     * The caller is responsible for memory management
     */
    Wheel(uint8_t motorPort, ros::NodeHandle * nodeHandle);
    virtual ~Wheel();

    /**
     * Set the rotation speed of the motor.
     *
     * \param speed the relative speed on a 0 - 255 scale.
     */
    void setSpeed(int speed);
    long readEndoder();

protected:
    /**
     * Constructor which initialises controller pin and encoder reader.
     * Member variables must be set before calling.
     */
    Wheel();

    uint8_t mMotorPort;
    ros::NodeHandle * mNodeHandle;

private:
    long mTachoCount;

};

}

#endif //ROVER_ARDUINO_WHEEL_H
