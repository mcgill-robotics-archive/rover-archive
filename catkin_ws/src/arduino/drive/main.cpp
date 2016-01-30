#include <Arduino.h>

#include <Servo.h>
#include <SPI.h>
#include <Wire.h>

#include "MotorController/MotorConfig.h"
#include <MotorController/MotorController.h>
#include <MotorController/MAXON.h>
#include "SteeringWheel.h"

ros::NodeHandle nh;

void setup() { }

void loop()
{
    nh.initNode();
    drive::MotorConfig leftFrontConfig;

    drive::SteeringWheel leftFront(leftFrontConfig,3,&nh);
    SPI.begin();
    Wire.begin();

    leftFront.setSteeringAngle(40);
    leftFront.setSpeed(20);
    leftFront.readEndoder();

    drive::MotorConfig midLeftConfig;

    midLeftConfig.brakePin = 5;
    midLeftConfig.controllerType = drive::_MAXON;

    drive::Wheel middleLeft(midLeftConfig, &nh);
    middleLeft.setSpeed(150);
    while(true) delay(10);
    // TODO: rosify
}
