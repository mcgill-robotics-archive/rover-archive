#include <Arduino.h>
#include <SPI/SPI.h>
#include <ros.h>
#include "Encoder.h"
#include "PitchRollCompute.h"
#include "../PID_v1/PID_v1.h"
#include <tf/transform_broadcaster.h>
#include <TransformFrame/TransformConfig.h>
#include <TransformFrame/TransformSender.h>

ros::NodeHandle nodeHandle;
tf::TransformBroadcaster broadcaster;


arm::Encoder link1(10);
arm::Encoder link2(11);

arm::PitchRollCompute pitchRollCompute(&link1, &link2);
float * pitchRollLink1 = (float *) malloc(2 * sizeof(float));
double setPoint;
double output;

// input is sensor, output is motor command, set point is desired position (from ros)
PID pidLink1((double *) &pitchRollLink1[0], &setPoint, &output, 0, 0, 0, DIRECT);

arm::TransformConfig config;
arm::TransformSender sender(nodeHandle, config);

void setup() {
    SPI.begin();
    broadcaster.init(nodeHandle);
    sender.updateRotations(PI/2, 0, 0, 0, 0, 0);
    sender.sendTransforms();
}

void loop() {
    pitchRollCompute.compute(pitchRollLink1);
    pidLink1.Compute();
    analogWrite(12, (int) output);
    nodeHandle.spinOnce();
    delay(10);
}