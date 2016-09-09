//
// Created by david on 9/9/16.
//

#ifndef ROVER_ARDUINO_SCIENCE_H
#define ROVER_ARDUINO_SCIENCE_H

#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <Servo/Servo.h>
#include <arduino/sensor.h>

#define SWITCH_PUB_TIMEOUT 500
#define CELL_PUB_TIMEOUT 5

#define CLOSE_SERVO_SIGNAL 0
#define OPEN_SERVO_SIGNAL 1

#define AUGER_SERVO_CLOSE_ANGLE 75
#define AUGER_SERVO_OPEN_ANGLE 175

#define SOIL_SERVO_CLOSE_ANGLE 1
#define SOIL_SERVO_OPEN_ANGLE 145

#define ROCK_SERVO_CLOSE_ANGLE 0
#define ROCK_SERVO_OPEN_ANGLE 145

void handle_auger_servo_position(const std_msgs::Int16 &message);
void handle_soil_servo_position(const std_msgs::Int16 &message);
void handle_rock_servo_position(const std_msgs::Int16 &message);
void set_science_servo_position(Servo servo, int16_t servoInput, int openPositionValue, int closePositionValue);

void handle_auger_angular_velocity(const std_msgs::Int16 &message);
void handle_auger_vertical_velocity(const std_msgs::Int16 &message);
void sensorServiceCallback(const arduino::sensor::Request &request, arduino::sensor::Response &response);

int16_t last_command_auger_angular_velocity = 0; // How fast and in what direction to spin the auger
int16_t last_command_auger_vertical_velocity = 0; // How fast and in what direction to move the auger up or down
unsigned long switch_pub_time = 0;
unsigned long loadcell_pub_time = 0;


#endif //ROVER_ARDUINO_SCIENCE_H
