//
// Created by David Lavoie-Boutin on 2016-01-18.
//
#include "MotorConfig.h"
#include "MotorController.h"
#include "MAXON.h"
#include "DRV8308.h"
#include "Pololu.h"

using namespace drive;

MotorController * MotorController::createMotorController(MotorConfig motorConfig, ros::NodeHandle *nodeHandle) {
    if (motorConfig.controllerType == _MAXON)
    {
        return new MAXON(
                motorConfig.speedPin,
                motorConfig.directionPin,
                motorConfig.enablePin,
                motorConfig.data1Pin,
                motorConfig.data2Pin,
                motorConfig.feedbackPin,
                motorConfig.mode
        );
    }
    else if (motorConfig.controllerType == _DRV8308)
    {
        return new drive::DRV8308();
    }
    else if (motorConfig.controllerType == _POLOLU)
    {
        return new arm::Pololu(motorConfig.speedPin, motorConfig.brakePin, motorConfig.data1Pin, motorConfig.data2Pin, nodeHandle);
    }
    return NULL;
}

MotorController::MotorController() { }

MotorController::~MotorController() {

}
