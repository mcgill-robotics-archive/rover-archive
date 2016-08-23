//
// Created by David Lavoie-Boutin on 2016-05-27.
//

#include <Wire.h>
#include <ros.h>
#include "Barometer.h"
#include <arduino/sensor.h>
#include <arduino/LimitSwitchScience.h>
#include "Humidity.h"
#include <std_msgs/Int16.h>
#include "pins_auger.h"
#include "Servo.h"
#include "WindSensor.h"

#define SWITCH_PUB_TIMEOUT 500


unsigned long switch_pub_schedule = 0;
void sensorServiceCallback(const arduino::sensor::Request &request, arduino::sensor::Response &response);
void handle_auger_angular_velocity(const std_msgs::Int16 &message);
void handle_auger_vertical_velocity(const std_msgs::Int16 &message);

Servo tempProbServo;

arduino::LimitSwitchScience limitSwitchMsg;

int16_t last_command_auger_angular_velocity = 0; // How fast and in what direction to spin the auger
int16_t last_command_auger_vertical_velocity = 0; // How fast and in what direction to move the auger up or down
int16_t last_command_gate_position = 0;
int16_t last_command_prob_position = 0;

ros::Subscriber<std_msgs::Int16> augerVelocitySub("auger_velocity", &handle_auger_angular_velocity);
ros::Subscriber<std_msgs::Int16> augerPositionSub("auger_position", &handle_auger_vertical_velocity);
ros::Publisher limitSwitchPub("limit_switch", &limitSwitchMsg);

Barometer * barometer;
WindSensor * windSensor;

ros::NodeHandle nodeHandle;
ros::ServiceServer<arduino::sensor::Request, arduino::sensor::Response> sensorService ("sensor_server", &sensorServiceCallback);


/**
 * Arduino setup
 */
void setup(){
    // Up / Down auger velocity pins
    pinMode(PIN_AUGER_VERTICAL_VELOCITY_INA, OUTPUT);
    pinMode(PIN_AUGER_VERTICAL_VELOCITY_INB, OUTPUT);
    pinMode(PIN_AUGER_VERTICAL_VELOCITY_PWM, OUTPUT);

    // Spin velocity pins for the auger
    pinMode(PIN_AUGER_ANGULAR_VELOCITY_INA, OUTPUT);
    pinMode(PIN_AUGER_ANGULAR_VELOCITY_INB, OUTPUT);
    pinMode(PIN_AUGER_ANGULAR_VELOCITY_PWM, OUTPUT);

    // Limit switches for the auger
    pinMode(PIN_LIMIT_SWITCH_UP, INPUT_PULLUP);
    pinMode(PIN_LIMIT_SWITCH_DOWN, INPUT_PULLUP);

    tempProbServo.attach(PIN_TEMP_PROB_POSITION);

    // Initialize ROS interactions
    nodeHandle.initNode();
    nodeHandle.advertiseService(sensorService);
    nodeHandle.subscribe(augerVelocitySub);
    nodeHandle.subscribe(augerPositionSub);
    nodeHandle.advertise(limitSwitchPub);

    // Initialize interfaces for Science sensors
    barometer = new Barometer;
    windSensor = new WindSensor;
}

/**
 * Arduino loop
 */
void loop (){
    nodeHandle.spinOnce();

    // We need to check the limit switches explicitly in every loop
    if (!digitalRead(PIN_LIMIT_SWITCH_UP) && last_command_auger_vertical_velocity > 0){
        analogWrite(PIN_AUGER_VERTICAL_VELOCITY_PWM, 0);
    }
    if (!digitalRead(PIN_LIMIT_SWITCH_DOWN) && last_command_auger_vertical_velocity < 0) {
        analogWrite(PIN_AUGER_VERTICAL_VELOCITY_PWM, 0);
    }

    // Publish limit switch details
    if( switch_pub_schedule < millis() ){
        switch_pub_schedule += SWITCH_PUB_TIMEOUT;
        limitSwitchMsg.limit_switch_up = digitalRead(PIN_LIMIT_SWITCH_UP);
        limitSwitchMsg.limit_switch_down = digitalRead(PIN_LIMIT_SWITCH_DOWN);
        limitSwitchPub.publish(&limitSwitchMsg);
    }
}

void sensorServiceCallback(const arduino::sensor::Request &request, arduino::sensor::Response &response){
    response.pressure = barometer->get_pressure()/5.5238471703;
    response.altitude = barometer->get_altitude();
    response.ambiant_temperature = barometer->get_ambient_temperature();
    response.humidity = get_humidity();
    response.wind_speed = windSensor->get_wind_speed();
}

void handle_auger_angular_velocity(const std_msgs::Int16 &message){
    last_command_auger_angular_velocity = message.data;
    if(last_command_auger_angular_velocity > 0 ){
        digitalWrite(PIN_AUGER_ANGULAR_VELOCITY_INA, HIGH);
        digitalWrite(PIN_AUGER_ANGULAR_VELOCITY_INB, LOW);
        analogWrite(PIN_AUGER_ANGULAR_VELOCITY_PWM, min(last_command_auger_angular_velocity,255));
    } else if (last_command_auger_angular_velocity < 0){
        digitalWrite(PIN_AUGER_ANGULAR_VELOCITY_INA, LOW);
        digitalWrite(PIN_AUGER_ANGULAR_VELOCITY_INB, HIGH);
        analogWrite(PIN_AUGER_ANGULAR_VELOCITY_PWM, min(abs(last_command_auger_angular_velocity),255));
    } else {
        analogWrite(PIN_AUGER_ANGULAR_VELOCITY_PWM, 0);
    }
}

void handle_auger_vertical_velocity(const std_msgs::Int16 &message){
    last_command_auger_vertical_velocity = message.data;
    if(digitalRead(PIN_LIMIT_SWITCH_UP) && last_command_auger_vertical_velocity > 0 ){
        digitalWrite(PIN_AUGER_VERTICAL_VELOCITY_INA, HIGH);
        digitalWrite(PIN_AUGER_VERTICAL_VELOCITY_INB, LOW);
        analogWrite(PIN_AUGER_VERTICAL_VELOCITY_PWM, min(last_command_auger_vertical_velocity,255));
    } else if (last_command_auger_vertical_velocity < 0){
        digitalWrite(PIN_AUGER_VERTICAL_VELOCITY_INA, LOW);
        digitalWrite(PIN_AUGER_VERTICAL_VELOCITY_INB, HIGH);
        analogWrite(PIN_AUGER_VERTICAL_VELOCITY_PWM, min(abs(last_command_auger_vertical_velocity),255));
    } else {
        // If there is an unwanted condition, such as a limit switch being triggered
        analogWrite(PIN_AUGER_VERTICAL_VELOCITY_PWM, 0);
    }
}

void handle_gate_position(const std_msgs::Int16 & message){
    last_command_gate_position = message.data;
    if(digitalRead(PIN_LIMIT_SWITCH_UP) && last_command_gate_position > 0 ){
        digitalWrite(PIN_GATE_POSITION_INA, HIGH);
        digitalWrite(PIN_GATE_POSITION_INB, LOW);
        analogWrite(PIN_GATE_POSITION_PWM, min(last_command_gate_position,120));
    } else if (last_command_gate_position < 0){
        digitalWrite(PIN_GATE_POSITION_INA, LOW);
        digitalWrite(PIN_GATE_POSITION_INB, HIGH);
        analogWrite(PIN_GATE_POSITION_PWM, min(abs(last_command_gate_position),120));
    } else {
        analogWrite(PIN_GATE_POSITION_PWM, 0);
    }
}

void handle_temp_prob_position(const std_msgs::Int16 & message){
    last_command_prob_position = message.data;
    if (last_command_prob_position > 0 ){
        tempProbServo.writeMicroseconds(1500 + last_command_prob_position);
    } else if (digitalRead(PIN_LIMIT_SWITCH_PROB) && last_command_prob_position < 0){
        tempProbServo.writeMicroseconds(1500 + last_command_prob_position);
    } else {
        tempProbServo.writeMicroseconds(1500);
    }
}
