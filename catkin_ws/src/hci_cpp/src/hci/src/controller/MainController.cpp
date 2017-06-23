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
        mArmController(),
        mJoystickController(mMainView.getJoystickView()),
        dcdcController(nh)
{
    // Setup all the sub-controllers in the constructor
    mJoystickController.registerController(&mDriveController, "Drive");
    mJoystickController.registerController(&mArmController, "Arm");

    // Since ROS subscriber information is coming from different threads, we
    // need to register the data types defined in the application with the
    // QT backend. This does not apply to ros message types
    // (i.e. std_msgs::Float64 does not require this)
    qRegisterMetaType<DriveStatusData>("DriveStatusData");
    qRegisterMetaType<SteeringMode>("SteeringMode");
    qRegisterMetaType<JoystickData>("JoystickData");


    // Move ROS Controllers to new threads to enable subscriber spins
    {
        QThread *driveControllerThread = new QThread;                                                       // Create thread object
        mThreadList.append(driveControllerThread);                                                          // Record thread in list
        mDriveController.moveToThread(driveControllerThread);                                               // Assign QObject to thread
        connect(driveControllerThread, &QThread::started, &mDriveController, &DriveController::process);    // Connect intializing method
        driveControllerThread->start();                                                                     // Start thread

        QThread *dcdcControllerThread = new QThread;
        mThreadList.append(dcdcControllerThread);
        dcdcController.moveToThread(dcdcControllerThread);
        connect(dcdcControllerThread, &QThread::started, &dcdcController, &DCDCController::process);
        dcdcControllerThread->start();
    }

    // Connect all the stuff
    connect(&mDriveController, &DriveController::steeringModeUpdated, &mMainView, &MainView::updateSteeringMode);
    connect(&mDriveController, &DriveController::wheelStatusUpdated, &mMainView, &MainView::updateDriveStatus);
    connect(&mDriveController, &DriveController::motorEnableChanged, &mMainView, &MainView::setMotorEnable);
    connect(&mMainView, &MainView::steeringModeChanged, this, [this](SteeringMode mode) {mDriveController.updateSteeringMode(mode);}); // directly call the slot (somehow the standard connection does not work) please investigate

    connect(&mMainView, &MainView::joystickModeChanged, &mJoystickController, &JoystickController::setActiveController);

    connect(&dcdcController, &DCDCController::InputVoltageUpdated ,&mMainView, &MainView::setInputVoltage);
    connect(&dcdcController, &DCDCController::InputCurrentUpdated ,&mMainView, &MainView::setInputCurrent);
    connect(&dcdcController, &DCDCController::OutputVoltageUpdated ,&mMainView, &MainView::setOutputVoltage);
    connect(&dcdcController, &DCDCController::OutputCurrentUpdated ,&mMainView, &MainView::setOutputCurrent);
    connect(&dcdcController, &DCDCController::OutputPowerUpdated ,&mMainView, &MainView::setOutputPower);
    connect(&dcdcController, &DCDCController::TemperatureUpdated ,&mMainView, &MainView::setTemperature);

    // Open the window
    mMainView.show();

    ROS_INFO("MainController Initialized");
}

MainController::~MainController() {
    for (int i = 0; i < mThreadList.length(); i++)
    {
        delete mThreadList.at(i);
        mThreadList.removeAt(i);
    }
}
