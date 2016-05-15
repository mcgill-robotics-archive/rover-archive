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
arm_control::JointPosition jointPosition;
arm_control::EncoderPosition encoderPosition;
arm_control::EncoderPosition motorSpeed;

void handle_arm_velocity(const arm_control::JointVelocities & message);
void handle_arm_position(const arm_control::JointPosition & message);
void handle_control_mode(const arm_control::ControlMode & message);

Potentiometer basePitch(PITCH_1_PIN, false, 1, &nodeHandle);
arm::Encoder baseYaw(BASE_YAW_SS_PIN, true, Base_Scale_Factor, &nodeHandle);
arm::Encoder differential1encoderLeft(DIFF_1_LEFT_SS_PIN, false, &nodeHandle);
arm::Encoder differential1encoderRight(DIFF_1_RIGHT_SS_PIN, true, &nodeHandle);
arm::Encoder differential2encoderLeft(DIFF_2_LEFT_SS_PIN, false, &nodeHandle);
arm::Encoder differential2encoderRight(DIFF_2_RIGHT_SS_PIN, true, &nodeHandle);

arm::PitchRollCompute differential1(&differential1encoderLeft, &differential1encoderRight);
arm::PitchRollCompute differential2(&differential2encoderLeft, &differential2encoderRight);

//todo: find pid constants
PID baseYawPID( baseYawPosition, &baseYawOutput, baseYawSetPoint, 0, 0, 0, DIRECT);
PID pitch1PID( pitch1Position, &pitch1Output, pitch1SetPoint, 0, 0, 0, DIRECT);
PID diff1leftPID( diff1posLeft, &diff1leftOutput,  diff2setPointLeft, 0, 0, 0, DIRECT);
PID diff2leftPID( diff2posLeft, &diff2leftOutput,  diff2setPointLeft, 1, 0, 0, DIRECT);
PID diff1rightPID( diff1posRight, &diff1rightOutput, diff1setPointRight, 0, 0, 0, DIRECT);
PID diff2rightPID( diff2posRight, &diff2rightOutput, diff2setPointRight, 1, 0, 0, REVERSE);

arm::TransformConfig transformConfig;
arm::TransformSender sender(&nodeHandle, transformConfig);

ros::Subscriber<arm_control::JointPosition> angleSubscriber("/arm/setPoints", &handle_arm_position);
ros::Subscriber<arm_control::JointVelocities> arm_subscriber("/arm_velocities", &handle_arm_velocity);
ros::Subscriber<arm_control::ControlMode> mode_subscriber("/arm_mode", &handle_control_mode);

ros::ServiceServer<arduino::ram::Request, arduino::ram::Response> ramService("~free_ram", &RAM::freeRamCallback);

ros::Publisher armJointPublisher("/arm/joint_feedback", &jointPosition);
ros::Publisher armEncoderPublisher("/arm/encoder_position_feedback", &encoderPosition);
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

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE2);

    baseYawPID.SetMode(AUTOMATIC);
    baseYawPID.SetOutputLimits(-255, 255);
    pitch1PID.SetMode(AUTOMATIC);
    pitch1PID.SetOutputLimits(-255, 255);
    diff1leftPID.SetMode(AUTOMATIC);
    diff1leftPID.SetOutputLimits(-255, 255);
    diff2leftPID.SetMode(AUTOMATIC);
    diff2leftPID.SetOutputLimits(-255, 255);
    diff1rightPID.SetMode(AUTOMATIC);
    diff1rightPID.SetOutputLimits(-255, 255);
    diff2rightPID.SetMode(AUTOMATIC);
    diff2rightPID.SetOutputLimits(-255, 255);

    nodeHandle.advertiseService(ramService);
    nodeHandle.subscribe(arm_subscriber);
    nodeHandle.subscribe(angleSubscriber);
    nodeHandle.subscribe(mode_subscriber);
    nodeHandle.advertise(armJointPublisher);
    nodeHandle.advertise(armEncoderPublisher);
    nodeHandle.advertise(armMotorSpeedPublisher);

    sender.init(&armJointPublisher);
    nodeHandle.loginfo("Completed initialisation of arm controller");
}



void loop() {
    /**
     * read encoder values
     * update and publish tf frames
     * update PID output
     * update motor commands
     */

    *baseYawPosition = baseYaw.readPosition();
    *pitch1Position = basePitch.readPosition();
    differential1.compute(pitchLink1, rollLink1, diff1posLeft, diff1posRight);
    differential2.compute(pitchLink2, rollLink2, diff2posLeft, diff2posRight);

    encoderPosition.base_yaw = *baseYawPosition;
    encoderPosition.base_pitch = *pitch1Position;
    encoderPosition.diff_1_left = *diff1posLeft;
    encoderPosition.diff_1_right = *diff1posRight;
    encoderPosition.diff_2_left = *diff2posLeft;
    encoderPosition.diff_2_right = *diff2posRight;
    armEncoderPublisher.publish(&encoderPosition);

    //send radian representation of the angles to the tf sender
    sender.updateRotations(radians(*baseYawPosition), radians(*pitch1Position),
                           radians(*pitchLink1), radians(*rollLink1),
                           radians(*pitchLink2), radians(*rollLink2));
    sender.sendTransforms();

    if (pid) {
        nodeHandle.logdebug("Compute PID");

        baseYawPID.Compute();
        pitch1PID.Compute();
        diff1leftPID.Compute();
        diff2leftPID.Compute();
        diff1rightPID.Compute();
        diff2rightPID.Compute();

        motorSpeed.diff_1_left = diff1leftOutput;
        motorSpeed.diff_1_right = diff1rightOutput;
        motorSpeed.end_effector = endEffectorOutput;
        motorSpeed.base_yaw = baseYawOutput;
        motorSpeed.base_pitch = pitch1Output;
        motorSpeed.diff_2_left = diff2leftOutput;
        motorSpeed.diff_2_right = diff2rightOutput;

        armMotorSpeedPublisher.publish(&motorSpeed);

        baseYawMotor->setSpeed((int) baseYawOutput);
        basePitchMotor->setSpeed((int) pitch1Output);
        diff1leftMotor->setSpeed((int) diff1leftOutput);
        diff1rightMotor->setSpeed((int) diff1rightOutput);
        diff2leftMotor->setSpeed((int) diff2leftOutput);
        diff2rightMotor->setSpeed((int) diff2rightOutput);
        endEffectorMotor->setSpeed((int) endEffectorOutput);
    }
    else {
        baseYawMotor->setSpeed((int) baseYawOutputVel);
        basePitchMotor->setSpeed((int) pitch1OutputVel);
        diff1leftMotor->setSpeed((int) diff1Vel[0]);
        diff1rightMotor->setSpeed((int) diff1Vel[1]);
        diff2leftMotor->setSpeed((int) diff2Vel[0]);
        diff2rightMotor->setSpeed((int) diff2Vel[1]);
        endEffectorMotor->setSpeed((int) endEffectorOutputVel);
    }

    nodeHandle.spinOnce();
    delay(1);
}

void handle_arm_position(const arm_control::JointPosition & message) {
    nodeHandle.logdebug("Receive new position");
    *pitch1SetPoint = message.base_pitch;
    differential1.inversePosition(message.diff_1_pitch, message.diff_1_roll,
                                  diff1setPointLeft, diff1setPointRight);
    differential2.inversePosition(message.diff_2_pitch, message.diff_2_roll,
                                  diff2setPointLeft, diff2setPointRight);

    *baseYawSetPoint = message.base_yaw;
    endEffectorOutput = message.end_effector;
}

void handle_arm_velocity(const arm_control::JointVelocities & message){
    nodeHandle.logdebug("Receive new speed");
    baseYawOutputVel = message.base_yaw;
    pitch1OutputVel = message.base_pitch;
    differential1.inverseSpeed(message.diff_1_pitch, message.diff_1_roll, diff1Vel);
    differential2.inverseSpeed(message.diff_2_pitch, message.diff_2_roll, diff2Vel);
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

