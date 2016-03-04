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

/**
 * Init ros
 * Advertise publishers
 * define callbacks
 * register subscribers
 * enter main loop
 * continuously update encoder positions and publish transforms
 */

ros::NodeHandle nodeHandle;
void handle_arm_velocity(const arm_control::JointVelocities & message);

arm::Encoder enfEffectorEncoder(END_EFFECTOR_SS_PIN, &nodeHandle);
arm::Encoder basePitch(PITCH_1_SS_PIN, &nodeHandle);
arm::Encoder differential1encoderLeft(DIFF_1_LEFT_SS_PIN, &nodeHandle);
arm::Encoder differential1encoderRight(DIFF_1_RIGHT_SS_PIN, &nodeHandle);
arm::Encoder differential2encoderLeft(DIFF_2_LEFT_SS_PIN, &nodeHandle);
arm::Encoder differential2encoderRight(DIFF_2_RIGHT_SS_PIN, &nodeHandle);

arm::PitchRollCompute differential1(&differential1encoderLeft, &differential1encoderRight);
arm::PitchRollCompute differential2(&differential2encoderLeft, &differential2encoderRight);

//PID baseYawPID((double *) &baseYawValue, &baseYawSetPoint, &baseYawOutput, 0, 0, 0, DIRECT);
//PID pitch1PID((double *) &pitch1Value, &pitch1SetPoint, &pitch1Output, 0, 0, 0, DIRECT);
//PID diff1leftPID((double *) &diff1pos[0], &diff1leftSetPoint, &diff1leftOutput, 0, 0, 0, DIRECT);
//PID diff2leftPID((double *) &diff2pos[0], &diff2leftSetPoint, &diff2leftOutput, 0, 0, 0, DIRECT);
//PID diff1rightPID((double *) &diff1pos[1], &diff1rightSetPoint, &diff1rightOutput, 0, 0, 0, DIRECT);
//PID diff2rightPID((double *) &diff2pos[1], &diff2rightSetPoint, &diff2rightOutput, 0, 0, 0, DIRECT);

arm::Motor endEffectorMotor(END_EFFECTOR_SPEED_PIN, -1, END_EFFECTOR_INA_PIN, END_EFFECTOR_INB_PIN, &nodeHandle);
arm::Motor pitch1Motor(PITCH_1_SPEED_PIN, PITCH_1_BRK_PIN, PITCH_1_INA_PIN, PITCH_1_INB_PIN, &nodeHandle);
arm::Motor diff_1_left(DIFF_1_LEFT_SPEED_PIN, DIFF_1_LEFT_BRK_PIN, DIFF_1_LEFT_INA_PIN, DIFF_1_LEFT_INB_PIN, &nodeHandle);
arm::Motor diff_1_right(DIFF_1_RIGHT_SPEED_PIN, DIFF_1_RIGHT_BRK_PIN, DIFF_1_RIGHT_INA_PIN, DIFF_1_RIGHT_INB_PIN, &nodeHandle);
arm::Motor diff_2_left(DIFF_2_LEFT_SPEED_PIN, DIFF_2_LEFT_BRK_PIN, DIFF_2_LEFT_INA_PIN, DIFF_2_LEFT_INB_PIN, &nodeHandle);
arm::Motor diff_2_right(DIFF_2_RIGHT_SPEED_PIN, DIFF_2_RIGHT_BRK_PIN, DIFF_2_RIGHT_INA_PIN, DIFF_2_RIGHT_INB_PIN, &nodeHandle);

arm::TransformConfig transformConfig;
arm::TransformSender sender(nodeHandle, transformConfig);

// TODO: define callback and register subscriber
//ros::Subscriber<control_systems::ArmAngles> angleSubscriber("/arm/setPoints", &handleAngles);
ros::Subscriber<arm_control::JointVelocities> arm_subscriber("/arm_velocities", &handle_arm_velocity);
ros::ServiceServer<arduino::ram::Request, arduino::ram::Response> ramService("~free_ram",&freeRamCallback);

void setup() {
    /**
     * Init bus for encoders
     * Register tf broadcaster
     * Setup motors and brakes
     */

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE2);

    nodeHandle.initNode();
    nodeHandle.advertiseService(ramService);
    nodeHandle.subscribe(arm_subscriber);
    sender.init();
    nodeHandle.loginfo("Completed initialisation of arm controller");
}


void loop() {
    /**
     * read encoder values
     * update PID output
     * TODO: update motor commands
     * update and publish tf frames
     */

    baseYawValue = 0;
    pitch1Value = basePitch.readPosition();
    differential1.compute(pitchRollLink1);
    differential2.compute(pitchRollLink2);

    sender.updateRotations(baseYawValue, pitch1Value, pitchRollLink1[0], pitchRollLink1[1], pitchRollLink2[0], pitchRollLink2[1]);
    sender.sendTransforms();

//    baseYawPID.Compute();
//    pitch1PID.Compute();
//    diff1leftPID.Compute();
//    diff2leftPID.Compute();
//    diff1rightPID.Compute();
//    diff2rightPID.Compute();

//    baseYawMotor.setSpeed(baseYawOutput);
    pitch1Motor.setSpeed(pitch1Output);
    diff_1_left.setSpeed(diff1pos[0]);
    diff_1_right.setSpeed(diff1pos[1]);
    diff_2_left.setSpeed(diff2pos[0]);
    diff_2_right.setSpeed(diff2pos[1]);
    endEffectorMotor.setSpeed(endEffectorOutput);



    // TODO: update motor commands

    nodeHandle.spinOnce();
    delay(1);
}

void handle_arm_velocity(const arm_control::JointVelocities & message){
    pitch1Output = message.base_pitch;
    differential1.inverse(message.diff_1_pitch, message.diff_1_roll, diff1pos);
    differential2.inverse(message.diff_2_pitch, message.diff_2_roll, diff2pos);
    endEffectorOutput = message.end_effector;
}