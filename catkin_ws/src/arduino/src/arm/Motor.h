//
// Created by David Lavoie-Boutin on 2016-02-08.
//

#ifndef ROVER_ARDUINO_MOTOR_H
#define ROVER_ARDUINO_MOTOR_H

#include <Arduino.h>
#include <ros.h>

namespace arm {

/**
 * \brief Class to drive a single brushed motor with brakes using the Pololu VNH5019 motor controller
 *
 * Drives the control pins to enable and disable the brakes along with the direction of the controllers.
 * Also sets the PWM duty cycle to achieve desired speed.
 */
class Motor {
public:

    /**
     * \brief Public constructor sets the proper pin direction and resets the speed to 0
     *
     * \param motorPin Pin number for PWM output
     * \param brakePin Pin number for brake digital output
     * \param INA Digital pin for control INA
     * \param INB Digital pin for control INB
     * \param nodeHandle Pointer to a valid ros::nodeHandle for logging debug information
     */
    Motor(uint8_t motorPin, uint8_t brakePin, uint8_t INA, uint8_t INB, ros::NodeHandle *nodeHandle);
    ~Motor();

    /**
     * \brief Engage the brakes
     */
    void lock();

    /**
     * \brief Set the speed of the motor
     *
     * Drives the PWM duty cycle, the brakes and the direction of the motor.
     * If the speed input is 0, the brakes will engage.
     *
     * \param speed Signed speed desired for this motor
     */
    void setSpeed(double speed);

    /**
     * \brief Disengage the breaks
     */
    void unlock();

    /**
     * Get the brakes status
     *
     * \return Will be true if the brakes are locked
     */
    bool isLocked();

    /**
     * \brief Get the current direction of the motor
     *
     * \return Will return true if the motor is running reverse (INA high and INB low)
     */
    bool isReverseDirection();

    /**
     * \brief Set the direction of the motor
     *
     * \param reverseDirection Whether or not the motor should run backwards
     */
    void setReverseDirection(bool reverseDirection);

private:
    uint8_t mMotorPin;
    uint8_t mBrakePin;
    uint8_t mINA;
    uint8_t mINB;
    ros::NodeHandle * mNh;
};

}

#endif //ROVER_ARDUINO_MOTOR_H
