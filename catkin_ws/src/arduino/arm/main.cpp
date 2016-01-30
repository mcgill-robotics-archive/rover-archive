#include <Arduino.h>
#include <SPI/SPI.h>
#include "Encoder.h"
#include "PitchRollCompute.h"
#include "../PID_v1/PID_v1.h"


arm::Encoder link1(10);
arm::Encoder link2(11);

arm::PitchRollCompute pitchRollCompute(&link1, &link2);
float * pitchRollLink1 = (float *) malloc(2 * sizeof(float));
double setPoint;
double output;

// input is sensor, output is motor command, set point is desired position (from ros)
PID pidLink1((double *) &pitchRollLink1[0], &setPoint, &output, 0, 0, 0, DIRECT);

void setup() {
    SPI.begin();
}

void loop() {
    pitchRollCompute.compute(pitchRollLink1);
    pidLink1.Compute();
    analogWrite(12, (int) output);
}