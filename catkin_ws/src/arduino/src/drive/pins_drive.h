//
// Created by David Lavoie-Boutin on 2016-02-27.
//

#ifndef ROVER_ARDUINO_PINS_H
#define ROVER_ARDUINO_PINS_H

#ifndef NC
#define NC 60
#endif

/*
 * pins.h
 * pins definitions for OTS drive system
 */
#ifdef MAXON_PINS

#define FL_DRIVE_PIN 7
#define FR_DRIVE_PIN 2
#define ML_DRIVE_PIN 9
#define MR_DRIVE_PIN 6
#define BL_DRIVE_PIN 4
#define BR_DRIVE_PIN 8

#define FL_DIRECTION_PIN 37
#define FR_DIRECTION_PIN 27
#define ML_DIRECTION_PIN 15
#define MR_DIRECTION_PIN 22
#define BL_DIRECTION_PIN 43
#define BR_DIRECTION_PIN 32

#define FL_ENABLE_PIN 39
#define FR_ENABLE_PIN 28
#define ML_ENABLE_PIN 16
#define MR_ENABLE_PIN 23
#define BL_ENABLE_PIN NC
#define BR_ENABLE_PIN 33

#define FL_STEERING_PIN 10
#define FR_STEERING_PIN 44
#define ML_SERVO NC
#define MR_SERVO 5
#define BL_STEERING_PIN 3
#define BR_STEERING_PIN 46

#define FL_DATA1_PIN 41
#define FL_DATA2_PIN 38

#define FR_DATA1_PIN 30
#define FR_DATA2_PIN 29

#define ML_DATA1_PIN 18
#define ML_DATA2_PIN 17

#define MR_DATA1_PIN 25
#define MR_DATA2_PIN 24

#define BL_DATA1_PIN 49
#define BL_DATA2_PIN 47

#define BR_DATA1_PIN 35
#define BR_DATA2_PIN 34

#define CAMERA_PAN_SERVO NC
#define CAMERA_TILT_SERVO NC

#define FL_READY_PIN 40
#define FR_READY_PIN 31
#define ML_READY_PIN 19
#define MR_READY_PIN 26
#define BL_READY_PIN 48
#define BR_READY_PIN 36

#endif //MAXON_PINS

#endif //ROVER_ARDUINO_PINS_H
