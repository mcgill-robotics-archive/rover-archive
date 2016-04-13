//
// Created by David Lavoie-Boutin on 2016-02-02.
//

#include "arm.h"
#include <TransformFrame/TransformSender.h>
#include "Encoder.h"
#include "PitchRollCompute.h"
#include <PID_v1/PID_v1.h>
#include <SPI.h>
#include "ram/ram.h"
#include "src/Pololu.h"
#include "pins_arm.h"
#include "arm_control/JointVelocities.h"
#include "arm_control/JointPosition.h"
#include "std_msgs/Float32.h"
#include "arm_control/ControlMode.h"

/**
 * Init ros
 * Advertise publishers
 * define callbacks
 * register subscribers
 * enter main loop
 * continuously update encoder positions and publish transforms
 */

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
std_msgs::Float32 ee_position;
arm_control::JointPosition jointPosition;
void handle_arm_velocity(const arm_control::JointVelocities & message);
void handle_arm_position(const arm_control::JointPosition & message);
void handle_control_mode(const arm_control::ControlMode & message);

arm::Encoder enfEffectorEncoder(END_EFFECTOR_SS_PIN, &nodeHandle);
arm::Encoder basePitch(PITCH_1_SS_PIN, &nodeHandle);
arm::Encoder baseYaw(BASE_YAW_SS_PIN, &nodeHandle);
arm::Encoder differential1encoderLeft(DIFF_1_LEFT_SS_PIN, &nodeHandle);
arm::Encoder differential1encoderRight(DIFF_1_RIGHT_SS_PIN, &nodeHandle);
arm::Encoder differential2encoderLeft(DIFF_2_LEFT_SS_PIN, &nodeHandle);
arm::Encoder differential2encoderRight(DIFF_2_RIGHT_SS_PIN, &nodeHandle);

arm::PitchRollCompute differential1(&differential1encoderLeft, &differential1encoderRight);
arm::PitchRollCompute differential2(&differential2encoderLeft, &differential2encoderRight);

//todo: find pid constants
PID baseYawPID((double *) &baseYawPosition, &baseYawSetPoint, &baseYawOutput, 0, 0, 0, DIRECT);
PID pitch1PID((double *) &pitch1Position, &pitch1SetPoint, &pitch1Output, 0, 0, 0, DIRECT);
PID diff1leftPID((double *) &diff1pos[0], (double *) &diff1setPoint[0], &diff1leftOutput, 0, 0, 0, DIRECT);
PID diff2leftPID((double *) &diff2pos[0], (double *) &diff2setPoint[0], &diff2leftOutput, 0, 0, 0, DIRECT);
PID diff1rightPID((double *) &diff1pos[1], (double *) &diff1setPoint[1], &diff1rightOutput, 0, 0, 0, DIRECT);
PID diff2rightPID((double *) &diff2pos[1], (double *) &diff2setPoint[1], &diff2rightOutput, 0, 0, 0, DIRECT);

arm::TransformConfig transformConfig;
arm::TransformSender sender(&nodeHandle, transformConfig);

ros::Subscriber<arm_control::JointPosition> angleSubscriber("/arm/setPoints", &handle_arm_position);
ros::Subscriber<arm_control::JointVelocities> arm_subscriber("/arm_velocities", &handle_arm_velocity);
ros::Subscriber<arm_control::ControlMode> mode_subscriber("/arm_mode", &handle_control_mode);
ros::ServiceServer<arduino::ram::Request, arduino::ram::Response> ramService("~free_ram",&RAM::freeRamCallback);
ros::Publisher eePublisher("/end_effector_position", &ee_position);
ros::Publisher armJointPublisher("/arm_joint_feedback", &jointPosition);

void setup() {
    /**
     * Init bus for encoders
     * Register tf broadcaster
     * Setup motors and brakes
     */

    nodeHandle.initNode();

    baseYawConfig.enablePin = BASE_YAW_ENABLE_PIN;
    baseYawConfig.data1Pin = BASE_YAW_DATA1_PIN;
    baseYawConfig.data2Pin = BASE_YAW_DATA2_PIN;
    baseYawConfig.directionPin = BASE_YAW_DIRECTION_PIN;
    baseYawConfig.feedbackPin = BASE_YAW_READY_PIN;
    baseYawConfig.speedPin = BASE_YAW_DRIVE_PIN;
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
    endEffectorConfig.brakePin = 60; // invalid pin on arduino

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

    nodeHandle.advertiseService(ramService);
    nodeHandle.subscribe(arm_subscriber);
    nodeHandle.subscribe(angleSubscriber);
    nodeHandle.subscribe(mode_subscriber);
    nodeHandle.advertise(eePublisher);
    nodeHandle.advertise(armJointPublisher);
    sender.init(&armJointPublisher);
    nodeHandle.loginfo("Completed initialisation of arm controller");
}


void loop() {
    /**
     * read encoder values
     * update PID output
     * update motor commands
     * update and publish tf frames
     */

    baseYawPosition = baseYaw.readPosition();
    endEffectorPosition = enfEffectorEncoder.readPosition();
    pitch1Position = basePitch.readPosition();
    differential1.compute(pitchRollLink1, diff1pos);
    differential2.compute(pitchRollLink2, diff2pos);

    ee_position.data = endEffectorPosition;
    eePublisher.publish(&ee_position);
    sender.updateRotations(baseYawPosition, pitch1Position, pitchRollLink1[0], pitchRollLink1[1], pitchRollLink2[0], pitchRollLink2[1]);
    sender.sendTransforms();

    if (pid) {
        baseYawPID.Compute();
        pitch1PID.Compute();
        diff1leftPID.Compute();
        diff2leftPID.Compute();
        diff1rightPID.Compute();
        diff2rightPID.Compute();

        baseYawMotor->setSpeed((int) baseYawOutput);
        basePitchMotor->setSpeed((int) pitch1Output);
        diff1leftMotor->setSpeed((int) diff1leftOutput);
        diff1rightMotor->setSpeed((int) diff1rightOutput);
        diff2leftMotor->setSpeed((int) diff2leftOutput);
        diff2rightMotor->setSpeed((int) diff2rightOutput);
        endEffectorMotor->setSpeed((int) endEffectorOutput);
    } else {
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
    pitch1SetPoint = message.base_pitch;
    differential1.inverse(message.diff_1_pitch, message.diff_1_roll, diff1setPoint);
    differential2.inverse(message.diff_2_pitch, message.diff_2_roll, diff2setPoint);
    baseYawSetPoint = message.base_yaw;
    endEffectorOutput = message.end_effector;
}

void handle_arm_velocity(const arm_control::JointVelocities & message){
    baseYawOutputVel = message.base_yaw;
    pitch1OutputVel = message.base_pitch;
    differential1.inverse(message.diff_1_pitch, message.diff_1_roll, diff1Vel);
    differential2.inverse(message.diff_2_pitch, message.diff_2_roll, diff2Vel);
    endEffectorOutputVel = message.end_effector;
}

void handle_control_mode(const arm_control::ControlMode & message){
    if (message.Position) pid = true;
    else pid = false;
}

