//
// Created by David Lavoie-Boutin on 23/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "ArmController.h"
#include <QDebug>
#include <ros/node_handle.h>
#include <arm_control/JointVelocities.h>

ArmController::ArmController(ros::NodeHandle &nh): nodeHandle(nh) {
    mCommandPublisher = nodeHandle.advertise<arm_control::JointVelocities>("joint_velocitiy", 100);
}

void ArmController::handleJoystickData(JoystickData data) {
    qDebug() << "Joystick data received in Arm Controller";
    enableMotors(data.buttons[0]);

    if (data.buttons[6])
        changeArmMode(OPEN);
    else if (data.buttons[7])
        changeArmMode(CLOSED);

    if (armMode == OPEN)
    {
        if (data.buttons[8])
            changeOpenLoopJoint(BASE);
        else if (data.buttons[9])
            changeOpenLoopJoint(D1);
        else if (data.buttons[10])
            changeOpenLoopJoint(D2);
        else if (data.buttons[11])
            changeOpenLoopJoint(END_EFFECTOR);
        jointVelocities = arm_control::JointVelocities();
        if (activeJoint == BASE) {
            jointVelocities.base_yaw = data.a1;
            jointVelocities.base_pitch = data.a2;
        } else if (activeJoint == D1) {
            jointVelocities.diff_1_pitch = data.a2;
            jointVelocities.diff_1_roll = data.a1;
        } else if (activeJoint == D2) {
            jointVelocities.diff_2_pitch = data.a2;
            jointVelocities.diff_2_roll = data.a1;
        } else if (activeJoint == END_EFFECTOR){
            jointVelocities.end_effector = data.a2;
        }
    }
}

void ArmController::process() {
    ROS_INFO("Starting arm controller thread");
    ros::Rate rate(10);

    while (nodeHandle.ok())
    {
        publish();
        ros::spinOnce();
        rate.sleep();
    }
}

void ArmController::changeArmMode(ArmMode mode) {
    ROS_INFO("ArmController.cpp: Arm mode changed");
    if (armMode != mode)
    {
        armMode = mode;
        emit armModeChanged(mode);
    }

}

void ArmController::changeOpenLoopJoint(ArmJoint joint) {
    ROS_INFO("ArmController.cpp: Active joint changed");
    if (activeJoint != joint)
    {
        activeJoint = joint;
        emit armJointChanged(joint);
    }
}

void ArmController::changeCloseLoopMode(ArmClosedLoopMode mode) {
    ROS_WARN("Closed loop controller not used");
}

void ArmController::publish() {
    if (armMode == OPEN)
    {
        if (motorEnable)
            mCommandPublisher.publish(jointVelocities);
        else
        {
            arm_control::JointVelocities jointVelocitiesEmpty;
            mCommandPublisher.publish(jointVelocitiesEmpty);
        }
    }
    else if (armMode == CLOSED)
    {
    }
}

void ArmController::enableMotors(bool enable) {
    if (motorEnable != enable)
    {
        motorEnable = enable;
        emit motorEnableChanged(motorEnable);
        ROS_INFO("ArmController.cpp: Changed motor enable to %s", (motorEnable ? "true" : "false"));
    }
}
