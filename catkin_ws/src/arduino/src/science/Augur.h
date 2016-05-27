//
// Created by David Lavoie-Boutin on 2016-05-27.
//

#ifndef ROVER_ARDUINO_AUGUR_H
#define ROVER_ARDUINO_AUGUR_H

#include <ros.h>
#include <MotorController.h>
#include <Servo.h>
#include <std_msgs/Int8.h>
#include <arduino/drill.h>


class Augur {
public:
    Augur(ros::NodeHandle* nodeHandle);

    void activateAugurDrill();
    void deactivateDrill();
    void augurHeightCallback(const std_msgs::Int8 &message);
    void drillCallback(const arduino::drill::Request & request, arduino::drill::Response & response);

    static void heightCallbackStatic(const std_msgs::Int8 &message);
    static void drillCallbackStatic(const arduino::drill::Request & request, arduino::drill::Response & response);

private:
    motor::MotorController *drillMotor;
    motor::MotorController *augurHeightMotor;
    Servo turnTableServo; //todo: servo might be moved outside of class
    ros::NodeHandle * mNodeHandle;
    ros::Subscriber<std_msgs::Int8> * subscriberAugur;
    ros::ServiceServer<arduino::drill::Request, arduino::drill::Response> * drillService;


    const uint8_t drillSpeed = 1;
    const uint8_t drillINA = 2;
    const uint8_t drillINB = 3;

    const uint8_t augurHeightSpeed = 1;
    const uint8_t augurHeightINA = 2;
    const uint8_t augurHeightINB = 3;

    const uint8_t turnTableSpeed = 1;

    const uint8_t pin_augur_limit_switch_up = 11;
    const uint8_t pin_augur_limit_switch_down = 12;
    bool upLimitSwitch;
    bool downLimitSwitch;
};

static Augur *objectPtr;


#endif //ROVER_ARDUINO_AUGUR_H
