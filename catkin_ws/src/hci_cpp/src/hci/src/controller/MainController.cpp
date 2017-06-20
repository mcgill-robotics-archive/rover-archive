//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "MainController.h"

MainController::MainController(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        mNH(nh),
        mPrivateNH(pnh),
        mDriveController(nh)
{
    // Setup all the sub-controllers in the constructor

    // Connect all the stuff
    connect(&mDriveController, &DriveController::steeringModeUpdated, &mMainView, &MainView::updateSteeringMode);
    connect(&mMainView, &MainView::steeringModeChanged, &mDriveController, &DriveController::updateSteeringMode);

    connect(&mDriveController, &DriveController::motorEnableChanged, &mMainView, &MainView::updatesEnabled);

    // Open the window
    mMainView.show();

    ROS_INFO("MainController Initialized");
}
