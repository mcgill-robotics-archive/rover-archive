#include <Arduino.h>
#include "ros.h"
#include "QuadEncoder/Encoder.h"
#include "std_msgs/Float64.h"

void cb_handle_command( const std_msgs::Float64& command );

ros::NodeHandle nh;

std_msgs::Float64 motor_position;

ros::Subscriber<std_msgs::Float64> command_sub("/command", &cb_handle_command);
ros::Publisher motor_pos("/state", &motor_position);

Encoder encoder(5,6);

int InA = 26;
int InB = 27;
int output_PWM = 2;

void setSpeed(double command) {
    
    if(command < 0.0) {
        digitalWrite(InA, HIGH);
        digitalWrite(InB, LOW);
    } else {
        digitalWrite(InA, LOW);
        digitalWrite(InB, HIGH);
    }

    analogWrite(output_PWM, (int)abs(command)%255);

}

void cb_handle_command(const std_msgs::Float64& command) {

    setSpeed(command.data);

}

void setup() {
    Serial.begin(57600);
    nh.initNode();
    nh.subscribe(command_sub);
    nh.advertise(motor_pos);

    pinMode(InA, OUTPUT);
    pinMode(InB, OUTPUT);
    pinMode(output_PWM, OUTPUT);
}

void loop() {
    motor_position.data = ((double)(encoder.read() % 1024) / 1024.0) * 2.0 * 3.14;   
    motor_pos.publish(&motor_position);
 
    nh.spinOnce();
    delay(1);
}

