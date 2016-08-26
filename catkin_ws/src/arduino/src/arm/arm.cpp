//
// Created by David Lavoie-Boutin on 2016-02-02.
//

#include <ros.h>
#include "arm.h"
#include "pins_arm.h"
#include "ram/ram.h"

#include <TransformFrame/TransformSender.h>
#include "Encoder.h"
#include "PitchRollCompute.h"
#include <PID_v1/PID_v1.h>
#include <SPI.h>
#include <include/MotorController.h>
#include "arm_control/JointVelocities.h"
#include "arm_control/JointPosition.h"
#include "arm_control/ControlMode.h"
#include "arm_control/EncoderPosition.h"
#include <arduino/LimitSwitchClaw.h>
#include "Potentiometer.h"

/**
 * Init ros
 * Advertise publishers
 * define callbacks
 * register subscribers
 * enter main loop
 * continuously update encoder positions and publish transforms
 */

const float Base_Scale_Factor = 63.0/20.0;

motor::MotorConfig baseYawConfig;
motor::MotorConfig basePitchConfig;
motor::MotorConfig diff1leftConfig;
motor::MotorConfig diff1rightConfig;
motor::MotorConfig diff2leftConfig;
motor::MotorConfig diff2rightConfig;
motor::MotorConfig endEffectorConfig;

motor::MotorController * baseYawMotor;
motor::MotorController * basePitchMotor;
motor::MotorController * diff1leftMotor;
motor::MotorController * diff1rightMotor;
motor::MotorController * diff2leftMotor;
motor::MotorController * diff2rightMotor;
motor::MotorController * endEffectorMotor;

ros::NodeHandle nodeHandle;
arm_control::EncoderPosition motorSpeed;
arduino::LimitSwitchClaw limitSwitchClawMsg;

void handle_arm_velocity(const arm_control::JointVelocities & message);
void handle_control_mode(const arm_control::ControlMode & message);

ros::Subscriber<arm_control::JointVelocities> arm_subscriber("/arm_velocities", &handle_arm_velocity);
ros::Subscriber<arm_control::ControlMode> mode_subscriber("/arm_mode", &handle_control_mode);
ros::Publisher clawPublisher("/claw_limit_switch", &limitSwitchClawMsg);
ros::ServiceServer<arduino::ram::Request, arduino::ram::Response> ramService("~free_ram", &RAM::freeRamCallback);

ros::Publisher armMotorSpeedPublisher("/arm/closed_loop_speed", &motorSpeed);

void setup() {
    /**
     * Init bus for encoders
     * Register tf broadcaster
     * Setup motors and brakes
     */

    nodeHandle.initNode();

    baseYawConfig.enablePin      = BASE_YAW_ENABLE_PIN;
    baseYawConfig.data1Pin       = BASE_YAW_DATA1_PIN;
    baseYawConfig.data2Pin       = BASE_YAW_DATA2_PIN;
    baseYawConfig.directionPin   = BASE_YAW_DIRECTION_PIN;
    baseYawConfig.feedbackPin    = BASE_YAW_READY_PIN;
    baseYawConfig.speedPin       = BASE_YAW_DRIVE_PIN;
    baseYawConfig.controllerType = motor::_MAXON;

    basePitchConfig.speedPin = PITCH_1_SPEED_PIN;
    basePitchConfig.data1Pin = PITCH_1_INA_PIN;
    basePitchConfig.data2Pin = PITCH_1_INB_PIN;
    basePitchConfig.brakePin = PITCH_1_BRK_PIN;

    diff1leftConfig.speedPin = DIFF_1_LEFT_SPEED_PIN;
    diff1leftConfig.data1Pin = DIFF_1_LEFT_INA_PIN;
    diff1leftConfig.data2Pin = DIFF_1_LEFT_INB_PIN;
    diff1leftConfig.brakePin = DIFF_1_LEFT_BRK_PIN;

    diff1rightConfig.speedPin = DIFF_1_RIGHT_SPEED_PIN;
    diff1rightConfig.data1Pin = DIFF_1_RIGHT_INA_PIN;
    diff1rightConfig.data2Pin = DIFF_1_RIGHT_INB_PIN;
    diff1rightConfig.brakePin = DIFF_1_RIGHT_BRK_PIN;

    diff2leftConfig.speedPin = DIFF_2_LEFT_SPEED_PIN;
    diff2leftConfig.data1Pin = DIFF_2_LEFT_INA_PIN;
    diff2leftConfig.data2Pin = DIFF_2_LEFT_INB_PIN;
    diff2leftConfig.brakePin = DIFF_2_LEFT_BRK_PIN;

    diff2rightConfig.speedPin = DIFF_2_RIGHT_SPEED_PIN;
    diff2rightConfig.data1Pin = DIFF_2_RIGHT_INA_PIN;
    diff2rightConfig.data2Pin = DIFF_2_RIGHT_INB_PIN;
    diff2rightConfig.brakePin = DIFF_2_RIGHT_BRK_PIN;

    endEffectorConfig.speedPin = END_EFFECTOR_SPEED_PIN;
    endEffectorConfig.data1Pin = END_EFFECTOR_INA_PIN;
    endEffectorConfig.data2Pin = END_EFFECTOR_INB_PIN;
    endEffectorConfig.brakePin = NC;

    basePitchConfig.controllerType   = motor::_POLOLU;
    diff1leftConfig.controllerType   = motor::_POLOLU;
    diff1rightConfig.controllerType  = motor::_POLOLU;
    diff2leftConfig.controllerType   = motor::_POLOLU;
    diff2rightConfig.controllerType  = motor::_POLOLU;
    endEffectorConfig.controllerType = motor::_POLOLU;

    basePitchMotor = motor::MotorController::createMotorController(basePitchConfig, &nodeHandle);
    baseYawMotor = motor::MotorController::createMotorController(baseYawConfig, &nodeHandle);
    diff1leftMotor = motor::MotorController::createMotorController(diff1leftConfig, &nodeHandle);
    diff1rightMotor = motor::MotorController::createMotorController(diff1rightConfig, &nodeHandle);
    diff2leftMotor = motor::MotorController::createMotorController(diff2leftConfig, &nodeHandle);
    diff2rightMotor = motor::MotorController::createMotorController(diff2rightConfig, &nodeHandle);
    endEffectorMotor = motor::MotorController::createMotorController(endEffectorConfig, &nodeHandle);
    baseYawMotor->enable(true);

    nodeHandle.advertiseService(ramService);
    nodeHandle.subscribe(arm_subscriber);
    nodeHandle.subscribe(mode_subscriber);
    nodeHandle.advertise(clawPublisher);
    nodeHandle.advertise(armMotorSpeedPublisher);

    nodeHandle.loginfo("Completed initialisation of arm controller");
}



void loop() {
    /**
     * read encoder values
     * update and publish tf frames
     * update PID output
     * update motor commands
     */

    baseYawMotor->setSpeed((int) baseYawOutputVel);
    basePitchMotor->setSpeed((int) pitch1OutputVel);
    diff1leftMotor->setSpeed((int) diff1Vel[0]);
    diff1rightMotor->setSpeed((int) diff1Vel[1]);
    diff2leftMotor->setSpeed((int) diff2Vel[0]);
    diff2rightMotor->setSpeed((int) diff2Vel[1]);
    endEffectorMotor->setSpeed((int) endEffectorOutputVel);

    // todo check which ss pin to use
    limitSwitchClawMsg.open = !digitalRead(END_EFFECTOR_SS_PIN);
    limitSwitchClawMsg.close = !digitalRead(BASE_YAW_SS_PIN);
    clawPublisher.publish(&limitSwitchClawMsg);

    nodeHandle.spinOnce();
    delay(1);
}

void handle_arm_velocity(const arm_control::JointVelocities & message){
    nodeHandle.logdebug("Receive new speed");
    baseYawOutputVel = message.base_yaw;
    pitch1OutputVel = message.base_pitch;
    arm::PitchRollCompute::inverseSpeed(message.diff_1_pitch, message.diff_1_roll, diff1Vel);
    arm::PitchRollCompute::inverseSpeed(message.diff_2_pitch, message.diff_2_roll, diff2Vel);
    endEffectorOutputVel = message.end_effector;
}

void handle_control_mode(const arm_control::ControlMode & message){
    if (message.Position) {
        pid = true;
        nodeHandle.logdebug("Switching to position control");
    }
    else {
        pid = false;
        nodeHandle.logdebug("Switching to speed control");
    }
}

