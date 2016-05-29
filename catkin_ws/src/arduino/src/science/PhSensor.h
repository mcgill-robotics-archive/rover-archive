//
// Created by David Lavoie-Boutin on 2016-05-27.
//

#ifndef ROVER_ARDUINO_PHSENSOR_H
#define ROVER_ARDUINO_PHSENSOR_H


#include <Arduino.h>
#include <ros.h>

class PhSensor {
public:
    PhSensor(ros::NodeHandle *nodeHandle);
    void serialEvent();
    float get_pH (char command);

private:
    char computerdata[20];           //we make a 20 byte character array to hold incoming data from a pc/mac/other.
    byte received_from_computer=0;   //we need to know how many characters have been received.
    byte serial_event=0;             //a flag to signal when data has been received from the pc/mac/other.
    byte code=0;                     //used to hold the I2C response code.
    char ph_data[20];                //we make a 20 byte character array to hold incoming data from the pH circuit.
    byte in_char=0;                  //used as a 1 byte buffer to store in bound bytes from the pH Circuit.
    byte i=0;                        //counter used for ph_data array.
    int time_=1800;                   //used to change the delay needed depending on the command sent to the EZO Class pH Circuit.
    float ph_float;                  //float var used to hold the float value of the pH.

    const int address = 99;  //for the ph sensor
    ros::NodeHandle * mNodeHandle;
};


#endif //ROVER_ARDUINO_PHSENSOR_H
