//
// Created by David Lavoie-Boutin on 2016-05-27.
//
#include "science.h"
#include <ros.h>
#include "Barometer.h"
#include <arduino/LimitSwitchScience.h>
#include <HX711/HX711.h>
#include "Humidity.h"
#include "pins_auger.h"
#include "WindSensor.h"
#include "DrillController.h"

Servo auger_servo, rock_servo, soil_servo;
arduino::LimitSwitchScience limitSwitchMsg;
std_msgs::Int64 sampleLoadcellMsg;

ros::Subscriber<std_msgs::Int16> augerServoSub("auger_servo_position", &handle_auger_servo_position);
ros::Subscriber<std_msgs::Int16> soilServoSub("soil_servo_position", &handle_soil_servo_position);
ros::Subscriber<std_msgs::Int16> rockServoSub("rock_servo_position", &handle_rock_servo_position);
ros::Subscriber<std_msgs::Int16> augerVelocitySub("auger_velocity", &handle_auger_angular_velocity);
ros::Subscriber<std_msgs::Int16> augerPositionSub("auger_position", &handle_auger_vertical_velocity);
ros::Publisher limitSwitchPub("limit_switch", &limitSwitchMsg);
ros::Publisher sampleLoadcellPublisher("sample_load_cell", &sampleLoadcellMsg);
ros::Publisher augerLoadcellPublisher("auger_load_cell", &sampleLoadcellMsg);

Barometer * barometer;
WindSensor * windSensor;
DrillController * drillController;
HX711 * augerLoadcell;
HX711 * sampleLoadcell;

ros::NodeHandle nodeHandle;
ros::ServiceServer<arduino::sensor::Request, arduino::sensor::Response> sensorService ("sensor_server", &sensorServiceCallback);

/**
 * Arduino setup
 */
void setup(){
    // Initialize servos to 0 state
    auger_servo.write(0);
    rock_servo.write(ROCK_SERVO_CLOSE_ANGLE);
    soil_servo.write(SOIL_SERVO_CLOSE_ANGLE);

    // Setup servo pins
    auger_servo.attach(PIN_AUGER_SERVO);
    rock_servo.attach(PIN_ROCK_SERVO);
    soil_servo.attach(PIN_SOIL_SERVO);

    // Limit switches for the auger
    pinMode(PIN_LIMIT_SWITCH_UP, INPUT_PULLUP);
    pinMode(PIN_LIMIT_SWITCH_DOWN, INPUT_PULLUP);

    // Initialize ROS interactions
    nodeHandle.initNode();

    drillController = new DrillController(&nodeHandle);
    barometer = new Barometer;
    windSensor = new WindSensor;
    sampleLoadcell = new HX711(24, 25);
    augerLoadcell = new HX711(22, 23);

    nodeHandle.advertiseService(sensorService);
    nodeHandle.subscribe(augerServoSub);
    nodeHandle.subscribe(soilServoSub);
    nodeHandle.subscribe(rockServoSub);
    nodeHandle.subscribe(augerVelocitySub);
    nodeHandle.subscribe(augerPositionSub);
    nodeHandle.advertise(limitSwitchPub);
    nodeHandle.advertise(sampleLoadcellPublisher);
    nodeHandle.advertise(augerLoadcellPublisher);

    // Initialize interfaces for Science sensors
    sampleLoadcell->tare();
    sampleLoadcell->set_scale(2280.f);
    sampleLoadcell->read();
}

/**
 * Arduino loop
 */
void loop (){
    nodeHandle.spinOnce();

    // We need to check the limit switches explicitly in every loop
    if (!digitalRead(PIN_LIMIT_SWITCH_UP) && last_command_auger_vertical_velocity < 0){
        drillController->setVerticalSpeed(0);
    }
    else if (!digitalRead(PIN_LIMIT_SWITCH_DOWN) && last_command_auger_vertical_velocity > 0) {
        drillController->setVerticalSpeed(0);
    }

    // Publish limit switch details
    if( switch_pub_time > SWITCH_PUB_TIMEOUT ) {
        switch_pub_time = 0;
        limitSwitchMsg.limit_switch_up = (bool) digitalRead(PIN_LIMIT_SWITCH_UP);
        limitSwitchMsg.limit_switch_down = (bool) digitalRead(PIN_LIMIT_SWITCH_DOWN);
        limitSwitchPub.publish(&limitSwitchMsg);
    }

    if (loadcell_pub_time > CELL_PUB_TIMEOUT)
    {
        loadcell_pub_time = 0;

        float load_t = ((abs(sampleLoadcell->get_units(1)))*5.0552);
        sampleLoadcellMsg.data = (int64_t) load_t;
        sampleLoadcellPublisher.publish(&sampleLoadcellMsg);

        load_t = ((abs(augerLoadcell->get_units(1)))*2);
        sampleLoadcellMsg.data = (int64_t) load_t;
        augerLoadcellPublisher.publish(&sampleLoadcellMsg);
    }

    switch_pub_time ++;
    loadcell_pub_time ++;
}

void sensorServiceCallback(const arduino::sensor::Request &request, arduino::sensor::Response &response){
    response.pressure = (int16_t) (barometer->get_pressure());
    response.altitude = (int16_t) barometer->get_altitude();
    response.ambiant_temperature = (int16_t) barometer->get_ambient_temperature();
    response.humidity = get_humidity();
    response.wind_speed = windSensor->get_wind_speed();
}

void handle_auger_angular_velocity(const std_msgs::Int16 &message){
    last_command_auger_angular_velocity = message.data;
    drillController->setDrillSpeed(last_command_auger_angular_velocity);
}

void handle_auger_vertical_velocity(const std_msgs::Int16 &message){
    last_command_auger_vertical_velocity = message.data;
    drillController->setVerticalSpeed(last_command_auger_vertical_velocity);
}

void handle_auger_servo_position(const std_msgs::Int16 &message){
    int16_t position = message.data;
    set_science_servo_position(auger_servo, position, AUGER_SERVO_OPEN_ANGLE, AUGER_SERVO_CLOSE_ANGLE);
}

void handle_soil_servo_position(const std_msgs::Int16 &message){
    int16_t position = message.data;
    set_science_servo_position(soil_servo, position, SOIL_SERVO_OPEN_ANGLE, SOIL_SERVO_CLOSE_ANGLE);
}

void handle_rock_servo_position(const std_msgs::Int16 &message){
    int16_t position = message.data;
    set_science_servo_position(rock_servo, position, ROCK_SERVO_OPEN_ANGLE, ROCK_SERVO_CLOSE_ANGLE);
}

/**
 * Helper for oppening or closing servos.
 *
 * @param servo: The particular servo that we want to open or close
 * @param servoInput: The value received by the subscriber
 * @param openPositionValue: The angle at which this particular servo is open
 * @param closePositionValue: The angle at which this particular servo is closed
 */
void set_science_servo_position(Servo servo, int16_t servoInput, int openPositionValue, int closePositionValue){
    if(servoInput == CLOSE_SERVO_SIGNAL){
        servo.write(closePositionValue);
    } else if(servoInput == OPEN_SERVO_SIGNAL){
        servo.write(openPositionValue);
    }
}
