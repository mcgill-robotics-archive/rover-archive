//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "DriveController.h"
#include <drive_control/DriveCommand.h>

DriveController::DriveController(ros::NodeHandle& nh) : mNodeHandle(nh){
    mCommandPublisher = mNodeHandle.advertise<drive_control::DriveCommand>("drive_command", 100);

    // TODO: Start timer loop to publish current status
}

void DriveController::updateSteeringMode(SteeringMode mode) {
    if (mSteeringMode != mode)
    {
        mSteeringMode = mode;
        emit steeringModeUpdated(mSteeringMode);
    }
}

void DriveController::enableMotors(bool enable) {
    if (enable != mMotorEnabled)
    {
        mMotorEnabled = enable;
        emit motorEnableChanged(mMotorEnabled);
    }

}

void DriveController::handleJoystickData(JoystickData &data) {
    // TODO: This class will eventually need to implement an abstract interface to receive joystick data when activated
}

void DriveController::publish() {
    drive_control::DriveCommand message;
    message.motion_enable = (uint8_t) mMotorEnabled;
    message.motion_ackerman = (uint8_t) (mSteeringMode == ACKERMANN);
    message.motion_pointsteer = (uint8_t) (mSteeringMode == POINT);
    message.motion_translatory = (uint8_t) (mSteeringMode == TRANSLATE);
    message.motion_skid = (uint8_t) (false);
    message.motion_swerve = (uint8_t) (false);

    // TODO: populate velocity command

    mCommandPublisher.publish(message) ;
}
