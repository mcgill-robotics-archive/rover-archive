//
// Created by David Lavoie-Boutin on 2016-05-27.
//

#include <Wire.h>
#include "PhSensor.h"

void PhSensor::serialEvent()            //this interrupt will trigger when the data coming from the serial monitor(pc/mac/other) is received.
{
    received_from_computer=Serial.readBytesUntil(13,computerdata,20); //we read the data sent from the serial monitor(pc/mac/other) until we see a <CR>. We also count how many characters have been received.
    computerdata[received_from_computer]=0;  //stop the buffer from transmitting leftovers or garbage.
    serial_event=1;
}

float PhSensor::get_pH(char command) {
    if(serial_event)            //if the serial_event=1.
    {
        if(computerdata[0]=='c'||computerdata[0]=='r')time_=1800; //if a command has been sent to calibrate or take a reading we wait 1800ms so that the circuit has time to take the reading.
        else time_=300;         //if any other command has been sent we wait only 300ms.


        Wire.beginTransmission(address); //call the circuit by its ID number.
        Wire.write(computerdata);        //transmit the command that was sent through the serial port.
        Wire.endTransmission();          //end the I2C data transmission.


        delay(time_);                    //wait the correct amount of time for the circuit to complete its instruction.

        Wire.requestFrom(address,20,1); //call the circuit and request 20 bytes (this may be more than we need)
        code=Wire.read();               //the first byte is the response code, we read this separately.

        switch (code)                  //switch case based on what the response code is.
        {
            case 1:                       //decimal 1.
                // means the command was successful.
                break;                        //exits the switch case.

            case 2:                        //decimal 2.
                mNodeHandle->loginfo("Failed");    //means the command has failed.
                break;                         //exits the switch case.

            case 254:                      //decimal 254.
                mNodeHandle->loginfo("Pending");   //means the command has not yet been finished calculating.
                break;                         //exits the switch case.

            case 255:                      //decimal 255.
                mNodeHandle->loginfo("No Data");   //means there is no further data to send.
                break;                         //exits the switch case.
        }

        while(Wire.available())          //are there bytes to receive.
        {
            in_char = Wire.read();           //receive a byte.
            ph_data[i]= in_char;             //load this byte into our array.
            i+=1;                            //incur the counter for the array element.
            if(in_char==0)                 //if we see that we have been sent a null command.
            {
                i=0;                        //reset the counter i to 0.
                Wire.endTransmission();     //end the I2C data transmission.
                break;                      //exit the while loop.
            }
        }

//		Serial.println(ph_data);          //print the data.
        serial_event=0;                   //reset the serial event flag.
    }

    //Uncomment this section if you want to take the pH value and convert it into floating point number.
    ph_float=atof(ph_data);
    return (ph_float);
}

PhSensor::PhSensor(ros::NodeHandle *nodeHandle) {
    mNodeHandle = nodeHandle;
    Wire.begin();
}




