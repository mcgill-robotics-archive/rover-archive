//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_MAINCONTROLLER_H
#define HCI_CPP_MAINCONTROLLER_H

#include <QObject>
#include <ros/ros.h>
#include <view/MainView.h>
#include "DriveController.h"
#include "JoystickController.h"
#include "DCDCController.h"
#include "ArmController.h"
#include "NavCameraController.h"


class MainController : public QObject {
    Q_OBJECT
public:
    MainController(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    virtual ~MainController();

private:
    ros::NodeHandle& mNH;
    ros::NodeHandle& mPrivateNH;

    MainView mMainView;

    DriveController mDriveController;
    ArmController mArmController;
    JoystickController mJoystickController;
    DCDCController dcdcController;
    NavCameraController navCameraController;
};


#endif //HCI_CPP_MAINCONTROLLER_H
