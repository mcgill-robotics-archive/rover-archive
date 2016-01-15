//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#include "Wheel.h"

void drive::Wheel::setSpeed(int speed) {
    analogWrite(mMotorPort, speed);
}

long drive::Wheel::readEndoder() {
    //TODO
    return mTachoCount;
}

drive::Wheel::Wheel() {
    mTachoCount = 0;
    pinMode(mMotorPort, OUTPUT);
    analogWrite(mMotorPort, 0);

    //TODO: init connection with encoder
}

drive::Wheel::~Wheel() {

}

drive::Wheel::Wheel(uint8_t motorPort, ros::NodeHandle * nodeHandle) {
    mNodeHandle = nodeHandle;
    mMotorPort = motorPort;
    Wheel();
    char init_message [48];
    sprintf(init_message, "Wheel initialized with drive pin %d", mMotorPort);

    mNodeHandle->logdebug(init_message);

}
