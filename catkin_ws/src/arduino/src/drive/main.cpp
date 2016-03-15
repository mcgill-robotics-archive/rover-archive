#include <Arduino.h>
#include "ros.h"

#include "MotorConfig.h"
#include "Wheel.h"
#include "SteeringWheel.h"
#include "ram/ram.h"
#include "rover_common/MotorStatus.h"
#include "rover_common/MotorControllerMode.h"
#include "std_msgs/Bool.h"
#include "drive_control/WheelCommand.h"
#include "pins_drive.h"

#define MOTOR_STATUS_UPDATE_RATE 100

void driveCallback( const drive_control::WheelCommand& setPoints );
void callbackMoving( const std_msgs::Bool& boolean);

ros::NodeHandle nh;
rover_common::MotorStatus motorStatusMessage;

ros::ServiceServer<arduino::ram::Request, arduino::ram::Response> ramService("~free_ram",&RAM::freeRamCallback);
ros::Publisher motorStatusPublisher("motor_status", &motorStatusMessage);
ros::Subscriber<std_msgs::Bool> movingSubscriber("/is_moving", &callbackMoving);
ros::Subscriber<drive_control::WheelCommand> driveSubscriber("/wheel_command", &driveCallback );

drive::MotorConfig configFL;
drive::MotorConfig configML;
drive::MotorConfig configBL;
drive::MotorConfig configFR;
drive::MotorConfig configMR;
drive::MotorConfig configBR;

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
    char message[10];
    sprintf(message, "%f", setPoints.flv);
    leftFront->setSpeed((int) setPoints.flv * 50);
    leftFront->setSteeringAngle((int) (90.0 + radToDeg(setPoints.flsa)));
    leftBack->setSpeed((int) setPoints.blv * 50);
    leftBack->setSteeringAngle((int) (90.0 + radToDeg(setPoints.blsa)));

    rightFront->setSpeed((int) -setPoints.frv * 50);
    rightFront->setSteeringAngle((int) (90.0 + radToDeg(setPoints.frsa)));
    rightBack->setSpeed((int) -setPoints.brv * 50);
    rightBack->setSteeringAngle((int) (90.0 + radToDeg(setPoints.brsa)));

    middleLeft->setSpeed((int) setPoints.mlv * 50);
    middleRight->setSpeed((int) -setPoints.mrv * 50);
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
    configFL.controllerType = drive::_MAXON;

    configML.enablePin = ML_ENABLE_PIN;
    configML.data1Pin = ML_DATA1_PIN;
    configML.data2Pin = ML_DATA2_PIN;
    configML.directionPin = ML_DIRECTION_PIN;
    configML.feedbackPin = ML_READY_PIN;
    configML.speedPin = ML_DRIVE_PIN;
    configML.controllerType = drive::_MAXON;

    configBL.enablePin = BL_ENABLE_PIN;
    configBL.data1Pin = BL_DATA1_PIN;
    configBL.data2Pin = BL_DATA2_PIN;
    configBL.directionPin = BL_DIRECTION_PIN;
    configBL.feedbackPin = BL_READY_PIN;
    configBL.speedPin = BL_DRIVE_PIN;
    configBL.controllerType = drive::_MAXON;

    configFR.enablePin = FR_ENABLE_PIN;
    configFR.data1Pin = FR_DATA1_PIN;
    configFR.data2Pin = FR_DATA2_PIN;
    configFR.directionPin = FR_DIRECTION_PIN;
    configFR.feedbackPin = FR_READY_PIN;
    configFR.speedPin = FR_DRIVE_PIN;
    configFR.controllerType = drive::_MAXON;

    configMR.enablePin = MR_ENABLE_PIN;
    configMR.data1Pin = MR_DATA1_PIN;
    configMR.data2Pin = MR_DATA2_PIN;
    configMR.directionPin = MR_DIRECTION_PIN;
    configMR.feedbackPin = MR_READY_PIN;
    configMR.speedPin = MR_DRIVE_PIN;
    configMR.controllerType = drive::_MAXON;

    configBR.enablePin = BR_ENABLE_PIN;
    configBR.data1Pin = BR_DATA1_PIN;
    configBR.data2Pin = BR_DATA2_PIN;
    configBR.directionPin = BR_DIRECTION_PIN;
    configBR.feedbackPin = BR_READY_PIN;
    configBR.speedPin = BR_DRIVE_PIN;
    configBR.controllerType = drive::_MAXON;

    leftFront = new drive::SteeringWheel(configFL, FL_STEERING_PIN, &nh);
    leftBack = new drive::SteeringWheel(configBL, BL_STEERING_PIN, &nh);
    rightFront = new drive::SteeringWheel(configFR, FR_STEERING_PIN, &nh);
    rightBack = new drive::SteeringWheel(configBR, BR_STEERING_PIN, &nh);

    middleLeft = new drive::Wheel(configML, &nh);
    middleRight = new drive::Wheel(configMR, &nh);
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
