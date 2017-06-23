//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "DriveController.h"
#include <drive_control/DriveCommand.h>
#include <QtCore/QTimer>

DriveController::DriveController(ros::NodeHandle& nh) : mNodeHandle(nh){
    mCommandPublisher = mNodeHandle.advertise<drive_control::DriveCommand>("drive_command", 100);
}

void DriveController::updateSteeringMode(SteeringMode mode) {
    if (mSteeringMode != mode)
    {
        std::string modeText;
        if (mode == ACKERMANN) modeText = ("ACKERMAN");
        if (mode == POINT) modeText = ("POINT");
        if (mode == TRANSLATE) modeText = ("TRANSLATE");
        ROS_INFO("DriveController.cpp: Steering mode changed to %s", modeText.c_str());

        mSteeringMode = mode;
        emit steeringModeUpdated(mSteeringMode);
    }
}

void DriveController::enableMotors(bool enable) {
    if (enable != mMotorEnabled)
    {
        mMotorEnabled = enable;
        emit motorEnableChanged(mMotorEnabled);
        ROS_INFO("DriveController.cpp: Motor enable status changed to %s", (enable ? "true":"false"));
    }
}

void DriveController::handleJoystickData(JoystickData data) {
    ROS_DEBUG("DriveController.cpp: Handling new joystick data");
    enableMotors(data.buttons[0]);

    // Set velocity depending if trigger is pressed.
    mLinearVel = data.buttons[0] ? data.a2 : 0;
    mAngularVel = data.buttons[0] ? data.a1 : 0;

    if (data.buttons[1])
        updateSteeringMode(ACKERMANN);
    else if (data.buttons[2])
        updateSteeringMode(POINT);
    else if (data.buttons[3])
        updateSteeringMode(TRANSLATE);
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
    ros::Rate rate(10);

    while(mNodeHandle.ok())
    {
        publish();
        ros::spinOnce();
        rate.sleep();
    }
    ROS_WARN("DriveController::process finishing");
}
