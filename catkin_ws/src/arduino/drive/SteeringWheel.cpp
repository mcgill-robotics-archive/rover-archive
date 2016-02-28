//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#include <Arduino.h>
#include "SteeringWheel.h"

namespace drive {

SteeringWheel::SteeringWheel(MotorConfig motorConfig, uint8_t servoPort, ros::NodeHandle *nodeHandle)
        : Wheel(motorConfig, nodeHandle) {
    mServoPort = servoPort;
    mMotorConfig = motorConfig;
    mNodeHandle = nodeHandle;

    mLowLimit = 1000;
    mHighLimit = 2000;

    pinMode(mServoPort, OUTPUT);

    if (!mServo.attached()) {
        mServo.attach(mServoPort);
    }

    char init_message [64];
    sprintf(init_message, "Steering wheel initialized with drive pin %d and "
            "servo pin %d", mMotorConfig.speedPin, mServoPort);

    mNodeHandle->logdebug(init_message);
}

SteeringWheel::~SteeringWheel() {
    mServo.detach();
}

void SteeringWheel::setSteeringAngle(int angle) {
    int servoCommand = (int) map(angle, 0, 180, 1000, 2000);
    servoCommand = constrain(servoCommand, mHighLimit, mLowLimit);

    mServo.write(servoCommand);

}

void SteeringWheel::setHighLimit(int HighLimit) {
    mHighLimit = HighLimit;
}

void SteeringWheel::setLowLimit(int LowLimit) {
    mLowLimit = LowLimit;
}

}