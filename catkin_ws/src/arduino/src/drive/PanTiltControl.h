//
// Created by Malcolm Watt on 26/08/16.
//

#ifndef ROVER_ARDUINO_PANTILTCONTROL_H
#define ROVER_ARDUINO_PANTILTCONTROL_H

#include "Arduino.h"
#include <ros.h>
#include "Servo.h"


namespace pan_tilt_control
{

/**
 * \brief Control both pan and tilt servos through a single interface.
 */
class PanTiltControl
{

private:
    const static int STOP_PAN_PW = 1500; // In microseconds, the pulsewidth required to stop the pan servo
    float currentPanPosition = 0;
    Servo panServo;
    Servo tiltServo;

public:
    /**
     * \brief Public constructor.
     *
     * \param panPin The pin for the panning servo.
     * \param tiltPin The pin for the tilting servo.
     */
    PanTiltControl(uint8_t panPin, uint8_t tiltPin, ros::NodeHandle &nodeHandle);
    virtual ~PanTiltControl();

    /**
     * \brief Sets the speed at which the servo rotates around the z axis.
     *
     * @param speed The speed at which to rotate the servo.
     */
    void setPanSpeed(float speed);

    /**
     * \brief Sets the speed at which the servo rotates around the x axis.
     *
     * @param speed The speed at which to rotate the servo.
     */
    void setTiltSpeed(float speed);
};

}

#endif //ROVER_ARDUINO_PANTILTCONTROL_H
