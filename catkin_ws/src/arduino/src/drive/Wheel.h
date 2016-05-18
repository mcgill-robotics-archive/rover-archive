//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#ifndef ROVER_ARDUINO_WHEEL_H
#define ROVER_ARDUINO_WHEEL_H

#include "Arduino.h"
#include <ros.h>
#include <MotorConfig.h>
#include <MotorController.h>

namespace drive {

/**
 * \brief Wrap a motor controller and an encoder counter in a single class.
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
    Wheel(motor::MotorConfig motorConfig, ros::NodeHandle * nodeHandle);
    virtual ~Wheel();

    /**
     * \brief Set the rotation speed of the motor.
     *
     * \param speed the relative speed on a -255 to 255 scale.
     */
    void setSpeed(float speed);

    /**
     * \brief Enable and disable the motor controller
     *
     * \param enable True if the motor controller should be on
     */
    void enable(bool enable);

    /**
     * \brief Get the current encoder position
     *
     * <b> NOT YET IMPLEMENTED</b>
     *
     * \return The current tachometer count since turn on.
     */
    long readEncoder();

    /**
     * \brief Stop the motor
     *
     * \param brk Boolean value true if to stop the motor
     */
    void brake(bool brk);

    /**
     * \brief Get the status of the motor controller
     *
     * \return True if operational
     */
    bool getStatus();

protected:
    /// NodeHandle for logging
    ros::NodeHandle * mNodeHandle;
    /// Configuration object for the motor controller
    motor::MotorConfig mMotorConfig;

private:
    long mTachoCount;
    motor::MotorController * mMotorController;

};

}

#endif //ROVER_ARDUINO_WHEEL_H
