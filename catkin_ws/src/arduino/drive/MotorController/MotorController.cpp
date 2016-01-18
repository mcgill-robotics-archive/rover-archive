//
// Created by David Lavoie-Boutin on 2016-01-18.
//

#include "MotorController.h"
#include "MAXON.h"
#include "DRV8308.h"

using namespace drive;

MotorController *MotorController::createMotorController(MotorConfig motorConfig) {
    if (motorConfig.controllerType == MotorConfig::ControllerType::MAXON)
    {
        return new MAXON();
    }
    else if (motorConfig.controllerType == MotorConfig::ControllerType::DRV8308)
    {
        return new DRV8308();
    }
    return NULL;
}

MotorController::MotorController() { }

MotorController::~MotorController() {

}
