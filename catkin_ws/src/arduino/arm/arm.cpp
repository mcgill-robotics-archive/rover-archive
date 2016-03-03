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

/**
 * Init ros
 * Advertise publishers
 * define callbacks
 * register subscribers
 * enter main loop
 * continuously update encoder positions and publish transforms
 */

ros::NodeHandle nodeHandle;

arm::Encoder baseYaw(BASE_YAW_SS_PIN, &nodeHandle);
arm::Encoder basePitch(PITCH_1_SS_PIN, &nodeHandle);
arm::Encoder differential1encoderLeft(DIFF_1_LEFT_SS_PIN, &nodeHandle);
arm::Encoder differential1encoderRight(DIFF_1_RIGHT_SS_PIN, &nodeHandle);
arm::Encoder differential2encoderLeft(DIFF_2_LEFT_SS_PIN, &nodeHandle);
arm::Encoder differential2encoderRight(DIFF_2_RIGHT_SS_PIN, &nodeHandle);

arm::PitchRollCompute differential1(&differential1encoderLeft, &differential1encoderRight);
arm::PitchRollCompute differential2(&differential2encoderLeft, &differential2encoderRight);

PID baseYawPID((double *) &baseYawValue, &baseYawSetPoint, &baseYawOutput, 0, 0, 0, DIRECT);
PID pitch1PID((double *) &pitch1Value, &pitch1SetPoint, &pitch1Output, 0, 0, 0, DIRECT);
PID pitch2PID((double *) &pitchRollLink1[0], &pitch2SetPoint, &pitch2Output, 0, 0, 0, DIRECT);
PID pitch3PID((double *) &pitchRollLink2[0], &pitch3SetPoint, &pitch3Output, 0, 0, 0, DIRECT);
PID roll1PID((double *) &pitchRollLink1[1], &roll1SetPoint, &roll1Output, 0, 0, 0, DIRECT);
PID roll2PID((double *) &pitchRollLink2[1], &roll2SetPoint, &roll2Output, 0, 0, 0, DIRECT);

arm::Motor baseYawMotor(BASE_YAW_SPEED_PIN, BASE_YAW_BRK_PIN, BASE_YAW_INA_PIN, BASE_YAW_INB_PIN);
arm::Motor pitch1Motor(PITCH_1_SPEED_PIN, PITCH_1_BRK_PIN, PITCH_1_INA_PIN, PITCH_1_INB_PIN);
arm::Motor diff_1_left(DIFF_1_LEFT_SPEED_PIN, DIFF_1_LEFT_BRK_PIN, DIFF_1_LEFT_INA_PIN, DIFF_1_LEFT_INB_PIN);
arm::Motor diff_1_right(DIFF_1_RIGHT_SPEED_PIN, DIFF_1_RIGHT_BRK_PIN, DIFF_1_RIGHT_INA_PIN, DIFF_1_RIGHT_INB_PIN);
arm::Motor diff_2_left(DIFF_2_LEFT_SPEED_PIN, DIFF_2_LEFT_BRK_PIN, DIFF_2_LEFT_INA_PIN, DIFF_2_LEFT_INB_PIN);
arm::Motor diff_2_right(DIFF_2_RIGHT_SPEED_PIN, DIFF_2_RIGHT_BRK_PIN, DIFF_2_RIGHT_INA_PIN, DIFF_2_RIGHT_INB_PIN);

arm::TransformConfig transformConfig;
arm::TransformSender sender(nodeHandle, transformConfig);

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
    sender.init();

    pinMode(13, OUTPUT);
}

// TODO: define callback and register subscriber
//ros::Subscriber<control_systems::ArmAngles> angleSubscriber("/arm/setPoints", &handleAngles);

void loop() {
    /**
     * read encoder values
     * update PID output
     * TODO: update motor commands
     * update and publish tf frames
     */


    digitalWrite(13, (uint8_t) !digitalRead(13));

    baseYawValue = baseYaw.readPosition();
    pitch1Value = basePitch.readPosition();
    differential1.compute(pitchRollLink1);
    differential2.compute(pitchRollLink2);

    baseYawPID.Compute();
    pitch1PID.Compute();
    pitch2PID.Compute();
    pitch3PID.Compute();
    roll1PID.Compute();
    roll2PID.Compute();

    baseYawMotor.setSpeed(baseYawOutput);
    pitch1Motor.setSpeed(pitch1Output);
    diff_1_left.setSpeed(pitch2Output);
    diff_1_right.setSpeed(pitch3Output);
    diff_2_left.setSpeed(roll1Output);
    diff_2_right.setSpeed(roll2Output);


    // TODO: update motor commands

    sender.updateRotations(baseYawValue, pitch1Value, pitchRollLink1[0], pitchRollLink1[1], pitchRollLink2[0], pitchRollLink2[1]);
    sender.sendTransforms();
    nodeHandle.spinOnce();
    delay(1);
}
