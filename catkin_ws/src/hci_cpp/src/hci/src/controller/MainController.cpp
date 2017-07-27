//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <rimstreamer/NyanVideoFeed.h>
#include "MainController.h"

MainController::MainController(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        mNH(nh),
        mPrivateNH(pnh),
        mMainView(),
        mDriveController(nh),
        mArmController(nh),
        mJoystickController(mMainView.getJoystickView()),
        dcdcController(nh),
        navCameraController(mMainView.getNavCamView()),
        navigationController(nh)
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
    qRegisterMetaType<ArmMode>("ArmMode");
    qRegisterMetaType<ArmJoint>("ArmJoint");
    qRegisterMetaType<ArmClosedLoopMode>("ArmClosedLoopMode");

    // Move ROS Controllers to new threads to enable subscriber spins
    {
        QThread *driveControllerThread = new QThread;                                                       // Create thread object
        mDriveController.moveToThread(driveControllerThread);                                               // Assign QObject to thread
        connect(driveControllerThread, &QThread::started, &mDriveController, &DriveController::process);    // Connect intializing method
        driveControllerThread->start();                                                                     // Start thread

        QThread *dcdcControllerThread = new QThread;
        dcdcController.moveToThread(dcdcControllerThread);
        connect(dcdcControllerThread, &QThread::started, &dcdcController, &DCDCController::process);
        dcdcControllerThread->start();

        QThread *armControllerThread = new QThread;
        mArmController.moveToThread(armControllerThread);
        connect(armControllerThread, &QThread::started, &mArmController, &ArmController::process);
        armControllerThread->start();
        
        QThread *navigationThread = new QThread;
        navigationController.moveToThread(navigationThread);
        connect(navigationThread, &QThread::started, &navigationController, &NavigationController::process);
        navigationThread->start();
    }

    // Connect all the stuff
    connect(&mDriveController, &DriveController::steeringModeUpdated, &mMainView, &MainView::updateSteeringMode);
    connect(&mDriveController, &DriveController::wheelStatusUpdated, &mMainView, &MainView::updateDriveStatus);
    connect(&mDriveController, &DriveController::motorEnableChanged, &mMainView, &MainView::setMotorEnable);
    connect(&mMainView, &MainView::steeringModeChanged, this, [this](SteeringMode mode) {mDriveController.updateSteeringMode(mode);}); // directly call the slot (somehow the standard connection does not work) please investigate

    connect(&mArmController, &ArmController::motorEnableChanged, &mMainView, &MainView::setMotorEnable);
    connect(&mArmController, &ArmController::armModeChanged , &mMainView, &MainView::setArmMode);
    connect(&mArmController, &ArmController::armJointChanged , &mMainView, &MainView::changeArmJoint);
    connect(&mMainView, &MainView::armJointChanged, this, [this](ArmJoint joint){mArmController.changeOpenLoopJoint(joint);});
    connect(&mMainView, &MainView::armModeChanged, this, [this](ArmMode armMode){mArmController.changeArmMode(armMode);});
    connect(&mMainView, &MainView::closeLoopModeChanged, this, [this](ArmClosedLoopMode mode){mArmController.changeCloseLoopMode(mode);});

    connect(&mMainView, &MainView::joystickModeChanged, &mJoystickController, &JoystickController::setActiveController);

    connect(&dcdcController, &DCDCController::InputVoltageUpdated ,&mMainView, &MainView::setInputVoltage);
    connect(&dcdcController, &DCDCController::InputCurrentUpdated ,&mMainView, &MainView::setInputCurrent);
    connect(&dcdcController, &DCDCController::OutputVoltageUpdated ,&mMainView, &MainView::setOutputVoltage);
    connect(&dcdcController, &DCDCController::OutputCurrentUpdated ,&mMainView, &MainView::setOutputCurrent);
    connect(&dcdcController, &DCDCController::OutputPowerUpdated ,&mMainView, &MainView::setOutputPower);
    connect(&dcdcController, &DCDCController::TemperatureUpdated ,&mMainView, &MainView::setTemperature);

    connect(&navigationController, &NavigationController::pitchChanged, &mMainView, &MainView::setPitch);
    connect(&navigationController, &NavigationController::rollChanged, &mMainView, &MainView::setRoll);
    connect(&navigationController, &NavigationController::yawChanged, &mMainView, &MainView::setYaw);
    connect(&navigationController, &NavigationController::longitudeChanged, &mMainView, &MainView::setLongitude);
    connect(&navigationController, &NavigationController::latitudeChanged, &mMainView, &MainView::setLatitude);

    // Open the window
    mMainView.show();

    ROS_INFO("MainController Initialized");



}

MainController::~MainController()
{
}
