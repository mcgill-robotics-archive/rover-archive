//
// Created by David Lavoie-Boutin on 2016-01-18.
//
#include "include/MotorConfig.h"
#include "include/MotorController.h"
#include "MAXON.h"
#include "DRV8308.h"
#include "Pololu.h"
#include "AfroESC.h"

using namespace motor;

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
        return new motor::DRV8308();
    }
    else if (motorConfig.controllerType == _POLOLU)
    {
        return new motor::Pololu(motorConfig.speedPin, motorConfig.brakePin, motorConfig.data1Pin, motorConfig.data2Pin, nodeHandle);
    }
    else if (motorConfig.controllerType == _AfroESC)
    {
        return new motor::AfroESC(motorConfig.speedPin);
    }
    return NULL;
}

MotorController::MotorController() { }

MotorController::~MotorController() {

}
