//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "DriveController.h"
#include <drive_control/DriveCommand.h>
#include <QDebug>

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
    // TODO: This class will eventually need to implement the abstract method and handle joystick data
}

void DriveController::publish() {
    drive_control::DriveCommand message;
    message.motion_enable = (uint8_t) mMotorEnabled;
    message.motion_ackerman = (uint8_t) (mSteeringMode == ACKERMANN);
    message.motion_pointsteer = (uint8_t) (mSteeringMode == POINT);
    message.motion_translatory = (uint8_t) (mSteeringMode == TRANSLATE);
    message.motion_skid = (uint8_t) (false);
    message.motion_swerve = (uint8_t) (false);
    message.velocity_command.linear.x = mLinearVel;
    message.velocity_command.angular.z = mAngularVel;

    mCommandPublisher.publish(message) ;
}

void DriveController::wheelStatusROSCallback(const rover_common::MotorStatus &message) {
    struct DriveStatusData data;
    data.flGood = message.fl;
    data.frGood = message.fr;
    data.mlGood = message.ml;
    data.mrGood = message.mr;
    data.blGood = message.bl;
    data.brGood = message.br;
    emit wheelStatusUpdated(data);
}

void DriveController::process()
{
    ROS_INFO("Starting drive controller thread");
    ros::Subscriber wheelStatusSub = mNodeHandle.subscribe("/motor_status", 1, &DriveController::wheelStatusROSCallback, this);
    while(mNodeHandle.ok())
    {
        ros::spinOnce();
    }
    ROS_WARN("DriveController::process finishing");
}
