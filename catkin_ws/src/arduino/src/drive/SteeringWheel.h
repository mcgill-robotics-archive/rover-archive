//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#ifndef ROVER_ARDUINO_STEERINGWHEEL_H
#define ROVER_ARDUINO_STEERINGWHEEL_H


#include <Servo.h>
#include "Wheel.h"

namespace drive {

/**
 * \brief Extension of the Wheel class to account for servos on the 4 corner
 * wheel assemblies.
 */
class SteeringWheel :  public Wheel{
public:
    /**
     * \brief Public constructor.
     *
     * Directly calls the Wheel constructor with the config and node handle objects
     *
     * \param motorConfig is the configuration object for the motor controller
     * \param servoPort is the pin number for the servo.
     * \param nodeHandle pointer to the node handle to use for logging.
     *
     * The caller is responsible for memory management
     */
    SteeringWheel(MotorConfig motorConfig, uint8_t servoPort, ros::NodeHandle * nodeHandle);
    virtual ~SteeringWheel();

    /**
     * \brief Set the steering angle of the servo
     *
     * \param angle The desired destination angle
     */
    void setSteeringAngle(int angle);

    /**
     * \brief Set the higher limit for the range of the servo
     *
     * \param HighLimit The limit in microseconds for the write command. Should never exceed 2000 for the megaservos
     */
    void setHighLimit(int HighLimit);

    /**
     * \brief Set the lower limit for the range of the servo
     *
     * \param LowLimit The limit in microseconds for the write command. Should never be less than 1000 for the megaservo
     */
    void setLowLimit(int LowLimit);

private:
    uint8_t mServoPort;
    Servo mServo;

    int mHighLimit;
    int mLowLimit;

};
}

#endif //ROVER_ARDUINO_STEERINGWHEEL_H
