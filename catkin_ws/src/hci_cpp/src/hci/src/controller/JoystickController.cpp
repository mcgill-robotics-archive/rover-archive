//
// Created by David Lavoie-Boutin on 20/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "JoystickController.h"
#include <QDebug>
#include <ros/ros.h>
#include <QThread>

JoystickController::JoystickController(JoystickView *joystickWidget) : mJoystickView(joystickWidget) {
    mHardwareJoystick = new HardwareJoystickController;
    if (mHardwareJoystick->joystickFound())
    {
        mType = Hardware;
        // start thread loop
        QThread *joystickThread = new QThread;
        mHardwareJoystick->moveToThread(joystickThread);
        connect(joystickThread, &QThread::started, mHardwareJoystick, &HardwareJoystickController::process);
        joystickThread->start();

    }
    else
    {
        mVirtualJoystick = new VirtualJoystick;
        mVirtualJoystick->show();
        mType = Software;
    }
}

void JoystickController::registerController(JoystickInterface *controller, QString name) {
    if (! mControllerHash.contains(name))
    {
        mControllerHash.insert(name, controller);
        mJoystickView->addMode(name);
    }
    else
    {
        throw std::runtime_error("Controller Name Already Used");
    }
}

void JoystickController::setActiveController(QString name) {
    if (mControllerHash.contains(name)) {
        if (mActiveController != mControllerHash[name]) {
            ROS_INFO("JoystickCintroller.cpp: Requested controller %s", name.toStdString().c_str());

            if (mType == Hardware)
            {
                // disconnect old controller
                disconnect(mHardwareJoystick, &HardwareJoystickController::joystickDataUpdated,
                           this, &JoystickController::joystickDataSlot);

                // connect new controller
                mActiveController = mControllerHash[name];
                connect(mHardwareJoystick, &HardwareJoystickController::joystickDataUpdated,
                        this, &JoystickController::joystickDataSlot);
            }

            else if (mType == Software)
            {
                // disconnect old controller
                disconnect(mVirtualJoystick, &VirtualJoystick::joystickDataUpdated,
                           this, &JoystickController::joystickDataSlot);

                // connect new controller
                mActiveController = mControllerHash[name];
                connect(mVirtualJoystick, &VirtualJoystick::joystickDataUpdated,
                        this, &JoystickController::joystickDataSlot);
            }

            emit activeControllerChanged(name);
        } else{
            ROS_INFO("JoystickController.cpp: Requesting same controller, ignoring");
        }
    }
    else
    {
        ROS_WARN("JoystickController.cpp: The requested joystick interface does not exist");
    }
}

void JoystickController::joystickDataSlot(JoystickData data) {
    mActiveController->handleJoystickData(data);
}
