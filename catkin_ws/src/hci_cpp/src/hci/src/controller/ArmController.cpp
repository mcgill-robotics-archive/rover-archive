//
// Created by David Lavoie-Boutin on 23/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "ArmController.h"
#include <QDebug>

ArmController::ArmController(){
    //TODO: Auto-Generated Constructor Stub
}

void ArmController::handleJoystickData(JoystickData data) {
    qDebug() << "Joystick data received in Arm Controller";
}
