//
// Created by David Lavoie-Boutin on 2016-02-02.
//

#include <Arduino.h>
#include <ros.h>
#include <TransformFrame/TransformSender.h>
#include "Encoder.h"
#include "PitchRollCompute.h"
#include "PID_v1/PID_v1.h"
#include <SPI.h>

/**
 * Init ros
 * Advertise publishers
 * define callbacks
 * register subscribers
 * enter main loop
 * continuously update encoder positions and publish transforms
 */

ros::NodeHandle nodeHandle;


arm::Encoder baseYaw(10, &nodeHandle);
arm::Encoder basePitch(11, &nodeHandle);
arm::Encoder differential1encoderLeft(12, &nodeHandle);
arm::Encoder differential1encoderRight(13, &nodeHandle);
arm::Encoder differential2encoderLeft(14, &nodeHandle);
arm::Encoder differential2encoderRight(15, &nodeHandle);

arm::PitchRollCompute differential1(&differential1encoderLeft, &differential1encoderRight);
arm::PitchRollCompute differential2(&differential2encoderLeft, &differential2encoderRight);

float * pitchRollLink1 = new float[2]; // first index is pitch, second is roll
float * pitchRollLink2 = new float[2]; // first index is pitch, second is roll
float baseYawValue = 0;
float pitch1Value = 0;

double baseYawSetPoint = 0;
double pitch1SetPoint = 0;
double pitch2SetPoint = 0;
double pitch3SetPoint = 0;
double roll1SetPoint = 0;
double roll2SetPoint = 0;

double baseYawOutput = 0;
double pitch1Output = 0;
double pitch2Output = 0;
double pitch3Output = 0;
double roll1Output = 0;
double roll2Output = 0;

PID baseYawPID((double *) &baseYawValue, &baseYawSetPoint, &baseYawOutput, 0, 0, 0, DIRECT);
PID pitch1PID((double *) &pitch1Value, &pitch1SetPoint, &pitch1Output, 0, 0, 0, DIRECT);
PID pitch2PID((double *) &pitchRollLink1[0], &pitch2SetPoint, &pitch2Output, 0, 0, 0, DIRECT);
PID pitch3PID((double *) &pitchRollLink2[0], &pitch3SetPoint, &pitch3Output, 0, 0, 0, DIRECT);
PID roll1PID((double *) &pitchRollLink1[1], &roll1SetPoint, &roll1Output, 0, 0, 0, DIRECT);
PID roll2PID((double *) &pitchRollLink2[1], &roll2SetPoint, &roll2Output, 0, 0, 0, DIRECT);

arm::TransformConfig config;
arm::TransformSender sender(nodeHandle, config);

void setup() {
    /**
     * TODO: Init bus for encoders
     * Register tf broadcaster
     * TODO: Setup motors and brakes
     */

    SPI.begin();
    broadcaster.init(nodeHandle);
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

    digitalWrite(13, !digitalRead(13));

//    baseYawValue = baseYaw.readPosition();
//    pitch1Value = basePitch.readPosition();
//    differential1.compute(pitchRollLink1);
//    differential2.compute(pitchRollLink2);

    baseYawPID.Compute();
    pitch1PID.Compute();
    pitch2PID.Compute();
    pitch3PID.Compute();
    roll1PID.Compute();
    roll2PID.Compute();

    // TODO: update motor commands

    sender.updateRotations(baseYawValue, pitch1Value, pitchRollLink1[0], pitchRollLink1[1], pitchRollLink2[0], pitchRollLink2[1]);
    sender.sendTransforms();
    delay(50);
}
