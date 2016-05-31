//
// Created by David Lavoie-Boutin on 2016-05-27.
//

#include "Augur.h"

void Augur::activateAugurDrill() {
    drillMotor->enable(true);
    drillMotor->setSpeed(255);
}

void Augur::deactivateDrill() {
    drillMotor->enable(false);
    drillMotor->setSpeed(0);
}

void Augur::augurHeightCallback(const std_msgs::Int8 &message) {
    //todo: possible scaling of the input data

    upLimitSwitch = !digitalRead(pin_augur_limit_switch_up);
    downLimitSwitch = !digitalRead(pin_augur_limit_switch_down);

    if (message.data < 0 && !downLimitSwitch) {
        augurHeightMotor->setSpeed(message.data);
    }
    else if (message.data > 0 && !upLimitSwitch) {
        augurHeightMotor->setSpeed(message.data);
    }
    else augurHeightMotor->setSpeed(0);
}

Augur::Augur(ros::NodeHandle *nodeHandle) {

    objectPtr = this;
    mNodeHandle = nodeHandle;

    motor::MotorConfig drillConfig;
    drillConfig.controllerType = motor::_POLOLU;
    drillConfig.data1Pin = drillINA;
    drillConfig.data2Pin = drillINB;
    drillConfig.speedPin = drillSpeed;

    motor::MotorConfig augurConfig;
    augurConfig.controllerType = motor::_POLOLU;
    augurConfig.data1Pin = augurHeightINA;
    augurConfig.data2Pin = augurHeightINB;
    augurConfig.speedPin = augurHeightSpeed;

    drillMotor = motor::MotorController::createMotorController(drillConfig, mNodeHandle);
    augurHeightMotor = motor::MotorController::createMotorController(augurConfig, mNodeHandle);
    turnTableServo.attach(turnTableSpeed);

    subscriberAugur = new ros::Subscriber<std_msgs::Int8>("", &Augur::heightCallbackStatic);
    drillService = new ros::ServiceServer<arduino::drill::Request, arduino::drill::Response> ("", &drillCallbackStatic);

    nodeHandle->subscribe(*subscriberAugur);
    nodeHandle->advertiseService(*drillService);

}

void Augur::drillCallback(const arduino::drill::Request &request, arduino::drill::Response &response) {
    if (request.activate) activateAugurDrill();
    else deactivateDrill();
    response.activated = request.activate;
}

void Augur::heightCallbackStatic(const std_msgs::Int8 &message) {
    if (objectPtr != NULL)
        objectPtr->augurHeightCallback(message);
}

void Augur::drillCallbackStatic(const arduino::drill::Request &request, arduino::drill::Response &response) {
    if (objectPtr != NULL)
        objectPtr->drillCallback(request, response);
}














