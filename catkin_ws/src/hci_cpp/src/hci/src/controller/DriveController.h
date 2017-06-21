//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_DRIVECONTROLLER_H
#define HCI_CPP_DRIVECONTROLLER_H

#include <QtWidgets/QWidget>
#include <ros/ros.h>
#include <model/DriveData.h>
#include "JoystickInterface.h"
#include <rover_common/MotorStatus.h>


class DriveController : public JoystickInterface {
Q_OBJECT
public:
    DriveController(ros::NodeHandle& nh);

    virtual ~DriveController() {};

    virtual void handleJoystickData(JoystickData& data);

public slots:
    void updateSteeringMode(SteeringMode mode);
    void enableMotors(bool enable);

    void process();

signals:
    void steeringModeUpdated(SteeringMode mode);
    void motorEnableChanged(bool enable);
    void wheelStatusUpdated(DriveStatusData status);

private:
    ros::NodeHandle& mNodeHandle;
    ros::Publisher mCommandPublisher;

    // Status data to be published
    SteeringMode mSteeringMode;
    bool mMotorEnabled;
    float mLinearVel;
    float mAngularVel;

private slots:
    void publish();
    void wheelStatusROSCallback(const rover_common::MotorStatus& message);
};


#endif //HCI_CPP_DRIVECONTROLLER_H
