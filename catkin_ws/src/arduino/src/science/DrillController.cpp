//
// Created by david on 9/9/16.
//

#include <MotorConfig.h>
#include "DrillController.h"
#include "pins_auger.h"

DrillController::DrillController(ros::NodeHandle *nodeHandle) {
    configHeightMotor.data1Pin = PIN_AUGER_VERTICAL_VELOCITY_INA;
    configHeightMotor.data2Pin = PIN_AUGER_VERTICAL_VELOCITY_INB;
    configHeightMotor.speedPin = PIN_AUGER_VERTICAL_VELOCITY_PWM;
    configHeightMotor.controllerType = motor::_POLOLU;

    configDrillMotor.data1Pin = PIN_AUGER_ANGULAR_VELOCITY_INA;
    configDrillMotor.data2Pin = PIN_AUGER_ANGULAR_VELOCITY_INB;
    configDrillMotor.speedPin = PIN_AUGER_ANGULAR_VELOCITY_PWM;
    configDrillMotor.controllerType = motor::_POLOLU;

    drillMotor = motor::MotorController::createMotorController(configDrillMotor, nodeHandle);
    heightMotor = motor::MotorController::createMotorController(configHeightMotor, nodeHandle);

    drillMotor->enable(true);
    heightMotor->enable(true);
}

void DrillController::setDrillSpeed(int speed) {
    drillMotor->setSpeed(speed);
}

void DrillController::setVerticalSpeed(int speed) {
    heightMotor->setSpeed(speed);
}
