//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "MainController.h"

MainController::MainController(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        mNH(nh),
        mPrivateNH(pnh),
        mMainView(),
        mDriveController(nh),
        mJoystickController(mMainView.getJoystickView())
{
    // Setup all the sub-controllers in the constructor
    mJoystickController.registerController(&mDriveController, "Drive");

    // Since ROS subscriber information is coming from different threads, we
    // need to register the data type with the QT backend
    qRegisterMetaType<DriveStatusData>("DriveStatusData");


    // Move ROS Controllers to new threads to enable subscriber spins
    QThread* driveControllerThread = new QThread;                                                       // Create thread object
    mDriveController.moveToThread(driveControllerThread);                                               // Assign QObject to thread
    connect(driveControllerThread, &QThread::started, &mDriveController, &DriveController::process);    // Connect intializing method
    driveControllerThread->start();                                                                     // Start thread

    // Connect all the stuff
    connect(&mDriveController, &DriveController::steeringModeUpdated, &mMainView, &MainView::updateSteeringMode);
    connect(&mDriveController, &DriveController::wheelStatusUpdated, &mMainView, &MainView::updateDriveStatus);
    connect(&mDriveController, &DriveController::motorEnableChanged, &mMainView, &MainView::updatesEnabled);
    connect(&mMainView, &MainView::steeringModeChanged, &mDriveController, &DriveController::updateSteeringMode);

    connect(&mMainView, &MainView::joystickModeChanged, &mJoystickController, &JoystickController::setActiveController);

    // Open the window
    mMainView.show();

    ROS_INFO("MainController Initialized");
}
