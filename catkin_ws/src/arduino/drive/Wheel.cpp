//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#include "Wheel.h"

void drive::Wheel::setSpeed(int speed) {
    mMotorController->setSpeed(speed);
}

long drive::Wheel::readEndoder() {
    //TODO
    return mTachoCount;
}

drive::Wheel::Wheel() {
    mTachoCount = 0;
    mMotorController = MotorController::createMotorController(mMotorConfig);
    mMotorController->enable(true);
    mMotorController->brake(true);

    //TODO: init connection with encoder
}

drive::Wheel::~Wheel() {
    delete mMotorController;
}

drive::Wheel::Wheel(MotorConfig motorConfig, ros::NodeHandle * nodeHandle) {
    mNodeHandle = nodeHandle;
    mMotorConfig = motorConfig;
    Wheel();
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
