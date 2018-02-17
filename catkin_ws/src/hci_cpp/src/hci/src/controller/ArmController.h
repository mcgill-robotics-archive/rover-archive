//
// Created by David Lavoie-Boutin on 23/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_ARMCONTROLLER_H
#define HCI_CPP_ARMCONTROLLER_H

#include <QtWidgets/QWidget>
#include <ros/ros.h>
#include <model/ArmData.h>
#include <arm_control/JointVelocities.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include "JoystickInterface.h"


class ArmController : public JoystickInterface {
Q_OBJECT
public:
    ArmController(ros::NodeHandle &nh);

    virtual ~ArmController() {};

    virtual void handleJoystickData(JoystickData data);

public slots:
    void process();
    void changeArmMode(ArmMode mode);
    void changeOpenLoopJoint(ArmJoint joint);
    void changeCloseLoopMode(ArmClosedLoopMode mode);

signals:
    void motorEnableChanged(bool enable);
    void armModeChanged(ArmMode mode);
    void armJointChanged(ArmJoint joint);
    void closedLoopModeChanged(ArmClosedLoopMode mode);

private:
    ros::NodeHandle& nodeHandle;
    ros::Publisher mCommandPublisher;
    ros::Publisher cCommandPublisher;
    
    bool motorEnable;
    arm_control::JointVelocities jointVelocities;
    geometry_msgs::Pose closeArm;
    ArmMode armMode;
    ArmJoint activeJoint;
    ArmClosedLoopMode closedLoopMode;
    void changeArmPoint(float a1, float a2, float a3);
    void changeArmQuad(float a1, float a2, float a3);

    void publish();
    void enableMotors(bool enable);
  
    float a1;
    float a2;
    float a3;
    geometry_msgs::Pose emptyPose;

};


#endif //HCI_CPP_ARMCONTROLLER_H
