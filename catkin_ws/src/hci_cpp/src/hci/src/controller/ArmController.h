//
// Created by David Lavoie-Boutin on 23/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_ARMCONTROLLER_H
#define HCI_CPP_ARMCONTROLLER_H

#include <QtWidgets/QWidget>
#include <ros/node_handle.h>
#include <model/ArmData.h>
#include <arm_control/JointVelocities.h>
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

private:
    ros::NodeHandle& nodeHandle;
    ros::Publisher mCommandPublisher;

    bool motorEnable;
    arm_control::JointVelocities jointVelocities;
    ArmMode armMode;
    ArmJoint activeJoint;
    ArmClosedLoopMode closedLoopMode;

    void publish();
    void enableMotors(bool enable);

};


#endif //HCI_CPP_ARMCONTROLLER_H
