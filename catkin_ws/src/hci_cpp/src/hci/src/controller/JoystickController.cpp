//
// Created by David Lavoie-Boutin on 20/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "JoystickController.h"
#include <QDebug>
JoystickController::JoystickController(JoystickView *joystickWidget) : mJoystickView(joystickWidget) {
    mJoystickAcquisition = new JoystickAcquisition;
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
    if (mControllerHash.contains(name) && mActiveController != mControllerHash[name])
    {
        // disconnect old controller
        disconnect(mJoystickAcquisition, &JoystickAcquisition::joystickDataUpdated,
                   mControllerHash[name], &JoystickInterface::handleJoystickData);

        // connect new controller
        mActiveController = mControllerHash[name];
        connect(mJoystickAcquisition, &JoystickAcquisition::joystickDataUpdated,
                mActiveController, &JoystickInterface::handleJoystickData);

        emit activeControllerChanged(name);
    }
    else
    {
        qDebug() << "The requested joystick does not exist";
    }
}
