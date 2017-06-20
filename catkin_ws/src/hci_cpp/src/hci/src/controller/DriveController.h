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


class DriveController : public QObject, JoystickInterface {
Q_OBJECT
public:
    DriveController(ros::NodeHandle& nh);

    virtual ~DriveController() {};

    virtual void handleJoystickData(JoystickData& data);

public slots:
    void updateSteeringMode(SteeringMode mode);
    void enableMotors(bool enable);

signals:
    void steeringModeUpdated(SteeringMode mode);
    void motorEnableChanged(bool enable);

private:
    ros::NodeHandle& mNodeHandle;
    ros::Publisher mCommandPublisher;

    // Status data to be published
    SteeringMode mSteeringMode;
    bool mMotorEnabled;

private slots:
    void publish();
};


#endif //HCI_CPP_DRIVECONTROLLER_H
