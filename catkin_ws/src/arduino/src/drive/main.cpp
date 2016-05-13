#include <Arduino.h>
#include "ros.h"

#include "MotorConfig.h"
#include "MotorController.h"
#include "Wheel.h"
#include "SteeringWheel.h"
#include "ram/ram.h"
#include "rover_common/MotorStatus.h"
#include "rover_common/MotorControllerMode.h"
#include "std_msgs/Bool.h"
#include "drive_control/WheelCommand.h"

#define MAXON_PINS
#define AFRO_CONTROLLERS

#include "pins_drive.h"


#define MOTOR_STATUS_UPDATE_RATE 100

void driveCallback( const drive_control::WheelCommand& setPoints );
void callbackMoving( const std_msgs::Bool& boolean);

ros::NodeHandle nh;
rover_common::MotorStatus motorStatusMessage;

ros::ServiceServer<arduino::ram::Request, arduino::ram::Response> ramService("~free_ram",&RAM::freeRamCallback);
ros::Publisher motorStatusPublisher("/motor_status", &motorStatusMessage);
ros::Subscriber<std_msgs::Bool> movingSubscriber("/is_moving", &callbackMoving);
ros::Subscriber<drive_control::WheelCommand> driveSubscriber("/wheel_command", &driveCallback );

motor::MotorConfig configFL;
motor::MotorConfig configML;
motor::MotorConfig configBL;
motor::MotorConfig configFR;
motor::MotorConfig configMR;
motor::MotorConfig configBR;

drive::SteeringWheel * leftFront;
drive::SteeringWheel *leftBack;
drive::SteeringWheel * rightFront;
drive::SteeringWheel * rightBack;
drive::Wheel * middleLeft;
drive::Wheel * middleRight;

unsigned long lastSend = 0;

float radToDeg(float rad)
{
    return rad / PI * 180.0;
}

void driveCallback( const drive_control::WheelCommand& setPoints )
{
    leftFront->setSpeed(setPoints.flv * 5);
    leftFront->setSteeringAngle((int) (90.0 + radToDeg(setPoints.flsa)));
    leftBack->setSpeed( setPoints.blv * 5);
    leftBack->setSteeringAngle((int) (90.0 + radToDeg(setPoints.blsa)));

    rightFront->setSpeed( -setPoints.frv * 5);
    rightFront->setSteeringAngle((int) (90.0 + radToDeg(setPoints.frsa)));
    rightBack->setSpeed( -setPoints.brv * 5);
    rightBack->setSteeringAngle((int) (90.0 + radToDeg(setPoints.brsa)));

    middleLeft->setSpeed(setPoints.mlv * 5);
    middleRight->setSpeed(-setPoints.mrv * 5);
}

void callbackMoving( const std_msgs::Bool& boolean)
{
    nh.logdebug("Entering moving");
    leftFront->enable(boolean.data);
    leftBack->enable(boolean.data);
    middleLeft->enable(boolean.data);
    rightBack->enable(boolean.data);
    rightFront->enable(boolean.data);
    middleRight->enable(boolean.data);
}

void setup() {
    nh.initNode();
    nh.advertiseService(ramService);
    nh.subscribe(movingSubscriber);
    nh.subscribe(driveSubscriber);
    nh.advertise(motorStatusPublisher);

    configFL.enablePin = FL_ENABLE_PIN;
    configFL.data1Pin = FL_DATA1_PIN;
    configFL.data2Pin = FL_DATA2_PIN;
    configFL.directionPin = FL_DIRECTION_PIN;
    configFL.feedbackPin = FL_READY_PIN;
    configFL.speedPin = FL_DRIVE_PIN;

    configML.enablePin = ML_ENABLE_PIN;
    configML.data1Pin = ML_DATA1_PIN;
    configML.data2Pin = ML_DATA2_PIN;
    configML.directionPin = ML_DIRECTION_PIN;
    configML.feedbackPin = ML_READY_PIN;
    configML.speedPin = ML_DRIVE_PIN;

    configBL.enablePin = BL_ENABLE_PIN;
    configBL.data1Pin = BL_DATA1_PIN;
    configBL.data2Pin = BL_DATA2_PIN;
    configBL.directionPin = BL_DIRECTION_PIN;
    configBL.feedbackPin = BL_READY_PIN;
    configBL.speedPin = BL_DRIVE_PIN;

    configFR.enablePin = FR_ENABLE_PIN;
    configFR.data1Pin = FR_DATA1_PIN;
    configFR.data2Pin = FR_DATA2_PIN;
    configFR.directionPin = FR_DIRECTION_PIN;
    configFR.feedbackPin = FR_READY_PIN;
    configFR.speedPin = FR_DRIVE_PIN;

    configMR.enablePin = MR_ENABLE_PIN;
    configMR.data1Pin = MR_DATA1_PIN;
    configMR.data2Pin = MR_DATA2_PIN;
    configMR.directionPin = MR_DIRECTION_PIN;
    configMR.feedbackPin = MR_READY_PIN;
    configMR.speedPin = MR_DRIVE_PIN;

    configBR.enablePin = BR_ENABLE_PIN;
    configBR.data1Pin = BR_DATA1_PIN;
    configBR.data2Pin = BR_DATA2_PIN;
    configBR.directionPin = BR_DIRECTION_PIN;
    configBR.feedbackPin = BR_READY_PIN;
    configBR.speedPin = BR_DRIVE_PIN;

#ifdef MAXON_CONTROLLERS
    configFL.controllerType = motor::_MAXON;
    configML.controllerType = motor::_MAXON;
    configBL.controllerType = motor::_MAXON;
    configFR.controllerType = motor::_MAXON;
    configMR.controllerType = motor::_MAXON;
    configBR.controllerType = motor::_MAXON;
#endif
#ifdef AFRO_CONTROLLERS
    configFL.controllerType = motor::_AfroESC;
    configML.controllerType = motor::_AfroESC;
    configBL.controllerType = motor::_AfroESC;
    configFR.controllerType = motor::_AfroESC;
    configMR.controllerType = motor::_AfroESC;
    configBR.controllerType = motor::_AfroESC;
#endif

    leftFront = new drive::SteeringWheel(configFL, FL_STEERING_PIN, &nh);
    leftBack = new drive::SteeringWheel(configBL, BL_STEERING_PIN, &nh);
    rightFront = new drive::SteeringWheel(configFR, FR_STEERING_PIN, &nh);
    rightBack = new drive::SteeringWheel(configBR, BR_STEERING_PIN, &nh);

    middleLeft = new drive::Wheel(configML, &nh);
    middleRight = new drive::Wheel(configMR, &nh);
    delay(0);
}

void sendMotorStatus(ros::Publisher &publisher) {
    motorStatusMessage.fl = leftFront->getStatus();
    motorStatusMessage.ml = middleLeft->getStatus();
    motorStatusMessage.bl = leftBack->getStatus();

    motorStatusMessage.fr = rightFront->getStatus();
    motorStatusMessage.mr = middleRight->getStatus();
    motorStatusMessage.br = rightBack->getStatus();
    publisher.publish(&motorStatusMessage);
}

void loop()
{

    if ((millis() - lastSend > MOTOR_STATUS_UPDATE_RATE))
    {
        sendMotorStatus(motorStatusPublisher);
        lastSend = millis();
    }

    nh.spinOnce();
    delay(1);
}
