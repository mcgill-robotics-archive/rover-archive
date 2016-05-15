//
// Created by David Lavoie-Boutin on 2016-03-03.
//

#ifndef ROVER_ARDUINO_PINS_ARM_H
#define ROVER_ARDUINO_PINS_ARM_H

#define NC 60

#define BASE_YAW_SS_PIN 31
#define BASE_YAW_ENABLE_PIN NC
#define BASE_YAW_DATA1_PIN NC
#define BASE_YAW_DATA2_PIN NC
#define BASE_YAW_DIRECTION_PIN NC
#define BASE_YAW_READY_PIN NC
#define BASE_YAW_DRIVE_PIN NC

#define END_EFFECTOR_SPEED_PIN 8
#define END_EFFECTOR_INA_PIN 28
#define END_EFFECTOR_INB_PIN 29
#define END_EFFECTOR_SS_PIN 31

#define PITCH_1_SPEED_PIN 10
#define PITCH_1_INA_PIN A9
#define PITCH_1_INB_PIN A10
#define PITCH_1_SS_PIN A12
#define PITCH_1_BRK_PIN A8

#define DIFF_1_LEFT_SPEED_PIN 2
#define DIFF_1_LEFT_INA_PIN 38
#define DIFF_1_LEFT_INB_PIN 39
#define DIFF_1_LEFT_SS_PIN 40
#define DIFF_1_LEFT_BRK_PIN 37

#define DIFF_1_RIGHT_SPEED_PIN 3
#define DIFF_1_RIGHT_INA_PIN 45
#define DIFF_1_RIGHT_INB_PIN 44
#define DIFF_1_RIGHT_SS_PIN 42
#define DIFF_1_RIGHT_BRK_PIN 46

#define DIFF_2_LEFT_SPEED_PIN 5
#define DIFF_2_LEFT_INA_PIN 33
#define DIFF_2_LEFT_INB_PIN 34
#define DIFF_2_LEFT_SS_PIN 35
#define DIFF_2_LEFT_BRK_PIN 32

#define DIFF_2_RIGHT_SPEED_PIN 4
#define DIFF_2_RIGHT_INA_PIN A14
#define DIFF_2_RIGHT_INB_PIN A15
#define DIFF_2_RIGHT_SS_PIN 48
#define DIFF_2_RIGHT_BRK_PIN A13

#endif //ROVER_ARDUINO_PINS_ARM_H
