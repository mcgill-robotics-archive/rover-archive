//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#include "Wheel.h"

void drive::Wheel::setSpeed(int speed) {
    mMotorController->setSpeed(speed);
}

long drive::Wheel::readEncoder() {
    //TODO
    return mTachoCount;
}

drive::Wheel::~Wheel() {
    delete mMotorController;
}

drive::Wheel::Wheel(motor::MotorConfig motorConfig, ros::NodeHandle * nodeHandle) {
    mNodeHandle = nodeHandle;
    mMotorConfig = motorConfig;
    mTachoCount = 0;
    mMotorController = motor::MotorController::createMotorController(mMotorConfig, nodeHandle);
    mMotorController->enable(true);
    mMotorController->brake(true);

    //TODO: init connection with encoder
    char init_message [48];
    sprintf(init_message, "Wheel initialized with drive pin %d", motorConfig.speedPin);
    mNodeHandle->logdebug(init_message);
}

void drive::Wheel::brake(bool brk) {
    mMotorController->brake(brk);
}

bool drive::Wheel::getStatus() {
    return mMotorController->getStatus();
}

void drive::Wheel::enable(bool enable) {
    mMotorController->enable(enable);
}
