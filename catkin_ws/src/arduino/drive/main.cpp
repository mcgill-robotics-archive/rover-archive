#include <Arduino.h>

#include <Servo.h>
#include "SteeringWheel.h"
Servo servo;
ros::NodeHandle nh;

drive::SteeringWheel leftFront(13, 7, &nh);

void setup()
{
    servo.attach(10);
}

int i = 0;

void loop()
{
    leftFront.setSteeringAngle(40);
    leftFront.setSpeed(20);
    leftFront.readEndoder();
    i ++;
    delay(10);
}
