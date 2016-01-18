#include <Arduino.h>

#include <Servo.h>
#include "MotorController/MotorConfig.h"
#include <MotorController/MotorController.h>
#include <MotorController/MAXON.h>
#include "SteeringWheel.h"
Servo servo;
ros::NodeHandle nh;

#include "SPI/SPI.h"

drive::SteeringWheel leftFront(13, 7, &nh);

void setup()
{
    servo.attach(10);
//    SPI.begin();
}

int i = 0;

void loop()
{
    leftFront.setSteeringAngle(40);
    leftFront.setSpeed(20);
    leftFront.readEndoder();
    i ++;

    drive::MotorConfig motorConfig;

    motorConfig.brakePin = 5;
    motorConfig.controllerType = drive::_MAXON;

    drive::MotorController * leftWheel = drive::MotorController::createMotorController(motorConfig);
//    drive::MAXON * maxon = static_cast<drive::MAXON*> (leftWheel);
    delay(10);
}
