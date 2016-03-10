//
// Created by David Lavoie-Boutin on 2016-02-02.
//

#include "arm.h"
#include <TransformFrame/TransformSender.h>
#include "Encoder.h"
#include "PitchRollCompute.h"
#include "PID_v1/PID_v1.h"
#include <SPI.h>
#include "../common/ram.h"
#include "Motor.h"
#include "pins_arm.h"
#include "arm_control/JointVelocities.h"
#include "arm_control/JointPosition.h"
#include "std_msgs/Float32.h"
#include "MotorConfig.h"
#include "MotorController.h"

/**
 * Init ros
 * Advertise publishers
 * define callbacks
 * register subscribers
 * enter main loop
 * continuously update encoder positions and publish transforms
 */
drive::MotorConfig baseConfig;
drive::MotorController * baseYawMotor;

ros::NodeHandle nodeHandle;
std_msgs::Float32 ee_position;
//void handle_arm_velocity(const arm_control::JointVelocities & message);
void handle_arm_position(const arm_control::JointPosition & message);

arm::Encoder enfEffectorEncoder(END_EFFECTOR_SS_PIN, &nodeHandle);
arm::Encoder basePitch(PITCH_1_SS_PIN, &nodeHandle);
arm::Encoder baseYaw(BASE_YAW_SS_PIN, &nodeHandle);
arm::Encoder differential1encoderLeft(DIFF_1_LEFT_SS_PIN, &nodeHandle);
arm::Encoder differential1encoderRight(DIFF_1_RIGHT_SS_PIN, &nodeHandle);
arm::Encoder differential2encoderLeft(DIFF_2_LEFT_SS_PIN, &nodeHandle);
arm::Encoder differential2encoderRight(DIFF_2_RIGHT_SS_PIN, &nodeHandle);

arm::PitchRollCompute differential1(&differential1encoderLeft, &differential1encoderRight);
arm::PitchRollCompute differential2(&differential2encoderLeft, &differential2encoderRight);

PID baseYawPID((double *) &baseYawPosition, &baseYawSetPoint, &baseYawOutput, 0, 0, 0, DIRECT);
PID pitch1PID((double *) &pitch1Position, &pitch1SetPoint, &pitch1Output, 0, 0, 0, DIRECT);
PID diff1leftPID((double *) &diff1pos[0], (double *) &diff1setPoint[0], &diff1leftOutput, 0, 0, 0, DIRECT);
PID diff2leftPID((double *) &diff2pos[0], (double *) &diff2setPoint[0], &diff2leftOutput, 0, 0, 0, DIRECT);
PID diff1rightPID((double *) &diff1pos[1], (double *) &diff1setPoint[1], &diff1rightOutput, 0, 0, 0, DIRECT);
PID diff2rightPID((double *) &diff2pos[1], (double *) &diff2setPoint[1], &diff2rightOutput, 0, 0, 0, DIRECT);

arm::Motor endEffectorMotor(END_EFFECTOR_SPEED_PIN, -1, END_EFFECTOR_INA_PIN, END_EFFECTOR_INB_PIN, &nodeHandle);
arm::Motor pitch1Motor(PITCH_1_SPEED_PIN, PITCH_1_BRK_PIN, PITCH_1_INA_PIN, PITCH_1_INB_PIN, &nodeHandle);
arm::Motor diff_1_left(DIFF_1_LEFT_SPEED_PIN, DIFF_1_LEFT_BRK_PIN, DIFF_1_LEFT_INA_PIN, DIFF_1_LEFT_INB_PIN, &nodeHandle);
arm::Motor diff_1_right(DIFF_1_RIGHT_SPEED_PIN, DIFF_1_RIGHT_BRK_PIN, DIFF_1_RIGHT_INA_PIN, DIFF_1_RIGHT_INB_PIN, &nodeHandle);
arm::Motor diff_2_left(DIFF_2_LEFT_SPEED_PIN, DIFF_2_LEFT_BRK_PIN, DIFF_2_LEFT_INA_PIN, DIFF_2_LEFT_INB_PIN, &nodeHandle);
arm::Motor diff_2_right(DIFF_2_RIGHT_SPEED_PIN, DIFF_2_RIGHT_BRK_PIN, DIFF_2_RIGHT_INA_PIN, DIFF_2_RIGHT_INB_PIN, &nodeHandle);

arm::TransformConfig transformConfig;
arm::TransformSender sender(nodeHandle, transformConfig);

// TODO: define callback and register subscriber
ros::Subscriber<arm_control::JointPosition> angleSubscriber("/arm/setPoints", &handle_arm_position);
//ros::Subscriber<arm_control::JointVelocities> arm_subscriber("/arm_velocities", &handle_arm_velocity);
ros::ServiceServer<arduino::ram::Request, arduino::ram::Response> ramService("~free_ram",&freeRamCallback);
ros::Publisher eePublisher("/end_effector_position", &ee_position);

void setup() {
    /**
     * Init bus for encoders
     * Register tf broadcaster
     * Setup motors and brakes
     */

    baseConfig.enablePin = BASE_YAW_ENABLE_PIN;
    baseConfig.data1Pin = BASE_YAW_DATA1_PIN;
    baseConfig.data2Pin = BASE_YAW_DATA2_PIN;
    baseConfig.directionPin = BASE_YAW_DIRECTION_PIN;
    baseConfig.feedbackPin = BASE_YAW_READY_PIN;
    baseConfig.speedPin = BASE_YAW_DRIVE_PIN;
    baseConfig.controllerType = drive::_MAXON;

    baseYawMotor = drive::MotorController::createMotorController(baseConfig);
    baseYawMotor->enable(true);

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE2);

    nodeHandle.initNode();
    nodeHandle.advertiseService(ramService);
//    nodeHandle.subscribe(arm_subscriber);
    nodeHandle.subscribe(angleSubscriber);
    nodeHandle.advertise(eePublisher);
    sender.init();
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

    baseYawPID.Compute();
    pitch1PID.Compute();
    diff1leftPID.Compute();
    diff2leftPID.Compute();
    diff1rightPID.Compute();
    diff2rightPID.Compute();

    baseYawMotor->setSpeed((int) baseYawOutput);
    pitch1Motor.setSpeed(pitch1Output);
    diff_1_left.setSpeed(diff1leftOutput);
    diff_1_right.setSpeed(diff1rightOutput);
    diff_2_left.setSpeed(diff2leftOutput);
    diff_2_right.setSpeed(diff2rightOutput);
    endEffectorMotor.setSpeed(endEffectorOutput);

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
//
//void handle_arm_velocity(const arm_control::JointVelocities & message){
//    pitch1Output = message.base_pitch;
//    differential1.inverse(message.diff_1_pitch, message.diff_1_roll, diff1pos);
//    differential2.inverse(message.diff_2_pitch, message.diff_2_roll, diff2pos);
//    endEffectorOutput = message.end_effector;
//}