//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#ifndef ROVER_ARDUINO_WHEEL_H
#define ROVER_ARDUINO_WHEEL_H

#include "Arduino.h"
#include <ros.h>
#include <MotorController/MotorConfig.h>
#include <MotorController/MotorController.h>

namespace drive {

/**
 * \brief Wrap the motor controller and the encoder counter in a single class.
 */
class Wheel {

public:
    /**
     * \brief Public constructor.
     *
     * The constructor instantiates a new motor controller with this configuration
     * and drives the outputs in a safe waiting state
     *
     * \param motorConfig Configuration object used for the motor controller
     * \param nodeHandle Pointer to the node handle to use for logging.
     */
    Wheel(MotorConfig motorConfig, ros::NodeHandle * nodeHandle);
    virtual ~Wheel();

    /**
     * Set the rotation speed of the motor.
     *
     * \param speed the relative speed on a 0 - 255 scale.
     */
    void setSpeed(int speed);
    void enable(bool enable);
    long readEncoder();
    void brake(bool brk);
    bool getStatus();

protected:
    ros::NodeHandle * mNodeHandle;
    MotorConfig mMotorConfig;

private:
    long mTachoCount;
    MotorController * mMotorController;

};

}

#endif //ROVER_ARDUINO_WHEEL_H
