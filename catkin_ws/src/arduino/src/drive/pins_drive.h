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

#define FR_DRIVE_PIN 4
#define FL_DRIVE_PIN 8
#define MR_DRIVE_PIN 9
#define ML_DRIVE_PIN 6
#define BL_DRIVE_PIN 7
#define BR_DRIVE_PIN 2

#define FR_DIRECTION_PIN 43
#define FL_DIRECTION_PIN 32 
#define MR_DIRECTION_PIN 15
#define ML_DIRECTION_PIN 22
#define BR_DIRECTION_PIN 37
#define BL_DIRECTION_PIN 27

#define FR_ENABLE_PIN NC
#define FL_ENABLE_PIN 33
#define MR_ENABLE_PIN 16
#define ML_ENABLE_PIN 23
#define BR_ENABLE_PIN 39
#define BL_ENABLE_PIN 28

#define FR_STEERING_PIN 3
#define FL_STEERING_PIN 46 
#define MR_SERVO 
#define ML_SERVO 
#define BR_STEERING_PIN 10
#define BL_STEERING_PIN 44

#define FR_DATA1_PIN 49
#define FR_DATA2_PIN 47

#define FL_DATA1_PIN 35
#define FL_DATA2_PIN 34

#define MR_DATA1_PIN 18
#define MR_DATA2_PIN 17

#define ML_DATA1_PIN 25
#define ML_DATA2_PIN 24

#define BR_DATA1_PIN 41
#define BR_DATA2_PIN 38

#define BL_DATA1_PIN 30
#define BL_DATA2_PIN 29

#define CAMERA_PAN_SERVO NC
#define CAMERA_TILT_SERVO NC

#define FR_READY_PIN 48
#define FL_READY_PIN 36
#define MR_READY_PIN 19
#define ML_READY_PIN 26
#define BR_READY_PIN 40
#define BL_READY_PIN 31

#endif //MAXON_PINS

#endif //ROVER_ARDUINO_PINS_H
