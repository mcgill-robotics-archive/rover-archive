//
// Created by david on 9/9/16.
//

#ifndef ROVER_ARDUINO_DRILLCONTROLLER_H
#define ROVER_ARDUINO_DRILLCONTROLLER_H


#include <MotorController.h>

class DrillController
{

public:
    DrillController(ros::NodeHandle *nodeHandle);

    void setDrillSpeed(int speed);
    void setVerticalSpeed(int speed);
private:
    motor::MotorConfig configDrillMotor;
    motor::MotorConfig configHeightMotor;

    motor::MotorController *drillMotor;
    motor::MotorController *heightMotor;
};


#endif //ROVER_ARDUINO_DRILLCONTROLLER_H
