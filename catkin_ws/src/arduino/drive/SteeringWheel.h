//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#ifndef ROVER_ARDUINO_STEERINGWHEEL_H
#define ROVER_ARDUINO_STEERINGWHEEL_H


#include <Servo/Servo.h>
#include "Wheel.h"

namespace drive {

class SteeringWheel :  public Wheel{
public:
    /**
     * Public constructor.
     * \param motorPort is the pin number to the motor controller
     * \param servoPort is the pin number for the servo.
     * \param nodeHandle pointer to the node handle to use for logging.
     * The caller is responsible for memory management
     */
    SteeringWheel(uint8_t motorPort, uint8_t servoPort, ros::NodeHandle * nodeHandle);
    virtual ~SteeringWheel();

    void setSteeringAngle(int angle);
    void setHighLimit(int HighLimit);
    void setLowLimit(int LowLimit);

protected:
    /**
     * Constructor which instantiates the servo object.
     * Member variables must be set before calling.
     */
    SteeringWheel();

private:
    uint8_t mServoPort;
    Servo mServo;

    int mHighLimit;
    int mLowLimit;

};
}

#endif //ROVER_ARDUINO_STEERINGWHEEL_H
