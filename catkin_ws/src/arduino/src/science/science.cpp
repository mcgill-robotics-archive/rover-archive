//
// Created by David Lavoie-Boutin on 2016-05-27.
//

#include <Wire.h>
#include <ros.h>
#include "Barometer.h"
#include "PhSensor.h"
#include <arduino/sensor.h>
#include <arduino/LimitSwitchScience.h>
#include "Sensors.h"
#include <std_msgs/Int16.h>
#include "pins_auger.h"
#include "Servo.h"

#define SWITCH_PUB_TIMEOUT 500


unsigned long switch_pub_schedule = 0;
void sensorServiceCallback(const arduino::sensor::Request &request, arduino::sensor::Response &response);
void handle_auger_velocity(const std_msgs::Int16 & message);
void handle_auger_position(const std_msgs::Int16 & message);
void handle_gate_position(const std_msgs::Int16 & message);
void handle_temp_prob_position(const std_msgs::Int16 & message);

Servo tempProbServo;

arduino::LimitSwitchScience limitSwitchMsg;

int16_t last_command_auger_velocity = 0;
int16_t last_command_auger_position = 0;
int16_t last_command_gate_position = 0;
int16_t last_command_prob_position = 0;

ros::Subscriber<std_msgs::Int16> augerVelocitySub("auger_velocity", &handle_auger_velocity);
ros::Subscriber<std_msgs::Int16> augerPositionSub("auger_position", &handle_auger_position);
ros::Subscriber<std_msgs::Int16> gatePositionSub("gate_position", &handle_gate_position);
ros::Subscriber<std_msgs::Int16> tempeProbPositionSub("temp_prob_position", &handle_temp_prob_position);
ros::Publisher limitSwitchPub("limit_switch", &limitSwitchMsg);

Barometer * barometer;
PhSensor * phSensor;

ros::NodeHandle nodeHandle;
ros::ServiceServer<arduino::sensor::Request, arduino::sensor::Response> sensorService ("sensor_server", &sensorServiceCallback);

void setup()
{
    pinMode(PIN_AUGER_POSITION_INA, OUTPUT);
    pinMode(PIN_AUGER_POSITION_INB, OUTPUT);
    pinMode(PIN_AUGER_POSITION_PWM, OUTPUT);
                    
    pinMode(PIN_AUGER_VELOCITY_INA, OUTPUT);
    pinMode(PIN_AUGER_VELOCITY_INB, OUTPUT);
    pinMode(PIN_AUGER_VELOCITY_PWM, OUTPUT);
    
    pinMode(PIN_GATE_POSITION_INA, OUTPUT);
    pinMode(PIN_GATE_POSITION_INB, OUTPUT);
    pinMode(PIN_GATE_POSITION_PWM, OUTPUT);
    
    pinMode(PIN_LIMIT_SWITCH_UP, INPUT_PULLUP);
    pinMode(PIN_LIMIT_SWITCH_DOWN, INPUT_PULLUP);
    pinMode(PIN_LIMIT_SWITCH_PROB, INPUT_PULLUP);
    pinMode(PIN_LIMIT_SWITCH_GATE, INPUT_PULLUP);
    
    tempProbServo.attach(PIN_TEMP_PROB_POSITION);
    nodeHandle.initNode();
    nodeHandle.advertiseService(sensorService);
    nodeHandle.subscribe(augerVelocitySub);
    nodeHandle.subscribe(augerPositionSub);
    nodeHandle.subscribe(gatePositionSub);
    nodeHandle.subscribe(tempeProbPositionSub);
    nodeHandle.advertise(limitSwitchPub);
    //things which have to be in the setup
    //barometer
    barometer = new Barometer;
    //pH sensor
    phSensor = new PhSensor(&nodeHandle);

    //auger
}

void loop ()
{
    nodeHandle.spinOnce();
    if (!digitalRead(PIN_LIMIT_SWITCH_UP) && last_command_auger_position > 0){
        analogWrite(PIN_AUGER_POSITION_PWM, 0);
    }
    if (!digitalRead(PIN_LIMIT_SWITCH_DOWN) && last_command_auger_position < 0) {
        analogWrite(PIN_AUGER_POSITION_PWM, 0);
    }
    if (!digitalRead(PIN_LIMIT_SWITCH_GATE) && last_command_gate_position > 0) {
        analogWrite(PIN_GATE_POSITION_PWM, 0);
    }
    if (!digitalRead(PIN_LIMIT_SWITCH_PROB) && last_command_prob_position < 0) {
        tempProbServo.writeMicroseconds(1500);
    }

    if( switch_pub_schedule < millis() ){
        switch_pub_schedule += SWITCH_PUB_TIMEOUT;
        limitSwitchMsg.limit_switch_up = digitalRead(PIN_LIMIT_SWITCH_UP);
        limitSwitchMsg.limit_switch_down = digitalRead(PIN_LIMIT_SWITCH_DOWN);
        limitSwitchMsg.limit_switch_prob = digitalRead(PIN_LIMIT_SWITCH_PROB);
        limitSwitchMsg.limit_switch_gate = digitalRead(PIN_LIMIT_SWITCH_GATE);
        limitSwitchPub.publish(&limitSwitchMsg);
    }
}

void sensorServiceCallback(const arduino::sensor::Request &request, arduino::sensor::Response &response) {
    response.ph = phSensor->get_pH('a');
    response.pressure = barometer->get_pressure()/5.5238471703;
    response.altitude = barometer->get_altitude();
    response.ambiant_temperature = barometer->get_ambient_temperature();
    response.ground_temperature = get_ground_temperature();
    response.humidity = get_humidity();

    }

void handle_auger_velocity(const std_msgs::Int16 & message){
    last_command_auger_velocity = message.data;
    if(last_command_auger_velocity > 0 ){
        digitalWrite(PIN_AUGER_VELOCITY_INA, HIGH);
        digitalWrite(PIN_AUGER_VELOCITY_INB, LOW);
        analogWrite(PIN_AUGER_VELOCITY_PWM, min(last_command_auger_velocity,255));
    } else if (last_command_auger_velocity < 0){
        digitalWrite(PIN_AUGER_VELOCITY_INA, LOW);
        digitalWrite(PIN_AUGER_VELOCITY_INB, HIGH);
        analogWrite(PIN_AUGER_VELOCITY_PWM, min(abs(last_command_auger_velocity),255));  
    } else { 
        analogWrite(PIN_AUGER_VELOCITY_PWM, 0);  
    }
}

void handle_auger_position(const std_msgs::Int16 & message){
    last_command_auger_position = message.data;
    if(digitalRead(PIN_LIMIT_SWITCH_UP) && last_command_auger_position > 0 ){
        digitalWrite(PIN_AUGER_POSITION_INA, HIGH);
        digitalWrite(PIN_AUGER_POSITION_INB, LOW);
        analogWrite(PIN_AUGER_POSITION_PWM, min(last_command_auger_position,255));
    } else if (last_command_auger_position < 0){
        digitalWrite(PIN_AUGER_POSITION_INA, LOW);
        digitalWrite(PIN_AUGER_POSITION_INB, HIGH);
        analogWrite(PIN_AUGER_POSITION_PWM, min(abs(last_command_auger_position),255));  
    } else { 
        analogWrite(PIN_AUGER_POSITION_PWM, 0);  
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

