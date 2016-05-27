//
// Created by David Lavoie-Boutin on 2016-05-27.
//



#include <Wire.h>
#include <ros.h>
#include "Barometer.h"
#include "PhSensor.h"
#include "Augur.h"
#include <arduino/sensor.h>
#include "Sensors.h"

void sensorServiceCallback(const arduino::sensor::Request &request, arduino::sensor::Response &response);
Barometer * barometer;
PhSensor * phSensor;
Augur * augur;
ros::NodeHandle * nodeHandle;
ros::ServiceServer<arduino::sensor::Request, arduino::sensor::Response> sensorService ("", &sensorServiceCallback);

void setup()
{
    nodeHandle = new ros::NodeHandle;
    nodeHandle->initNode();
    nodeHandle->advertiseService(sensorService);

    //things which have to be in the setup
    //barometer
    barometer = new Barometer;
    //pH sensor
    phSensor = new PhSensor(nodeHandle);

    //augur
    augur = new Augur(nodeHandle);

    pinMode(pin_augur_servo_limit_switch, INPUT_PULLUP);
}

void loop ()
{
    nodeHandle->spinOnce();
}

void sensorServiceCallback(const arduino::sensor::Request &request, arduino::sensor::Response &response) {
    response.ph = phSensor->get_pH('a');
    response.altitude = barometer->get_altitude();
    response.ambiant_temperature = barometer->get_ambient_temperature();
    response.pressure = barometer->get_pressure();
    response.ground_temperature = get_ground_temperature();
    response.humidity = get_humidity();
}

