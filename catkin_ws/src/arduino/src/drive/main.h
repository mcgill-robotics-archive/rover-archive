//
// Created by david on 9/8/16.
//

#ifndef ROVER_ARDUINO_MAIN_H
#define ROVER_ARDUINO_MAIN_H

#include "ros.h"
#include "drive_control/WheelCommand.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "Wheel.h"
#include "SteeringWheel.h"
#include "PanTiltControl.h"

#define MOTOR_STATUS_UPDATE_RATE 100
#define MOTOR_COMMAND_TIMEOUT 1000
#define MAXON_PINS
#define AFRO_CONTROLLERS

void driveCallback( const drive_control::WheelCommand& setPoints );
void callbackMoving( const std_msgs::Bool& boolean);
void panTiltCallback(const geometry_msgs::Twist& speeds);
void sendMotorStatus(ros::Publisher &publisher);
void stopMotor();
float radToDeg(float rad);

drive::SteeringWheel * leftFront;
drive::SteeringWheel *leftBack;
drive::SteeringWheel * rightFront;
drive::SteeringWheel * rightBack;
drive::Wheel * middleLeft;
drive::Wheel * middleRight;

pan_tilt_control::PanTiltControl * mastCameraController;

unsigned long lastSend = 0;
unsigned long lastCommandReceived = 0;

#endif //ROVER_ARDUINO_MAIN_H
