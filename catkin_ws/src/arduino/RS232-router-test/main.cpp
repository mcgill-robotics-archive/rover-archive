//
// Created by David Lavoie-Boutin on 2015-10-15.
//

//Created August 23 2006
//Heather Dewey-Hagborg
//http://www.arduino.cc

#include <ctype.h>
#include <Arduino.h>
#include "CustomSerial.h"

byte SWval;

void setup() {
    Serial.print("Start");
    pinMode(rx,INPUT);
    pinMode(tx,OUTPUT);
    digitalWrite(tx,HIGH);
    delay(2);
    digitalWrite(13,HIGH); //turn on debugging LED
    SWprint('h');  //debugging hello
    SWprint('i');
    SWprint(10); //carriage return
    Serial.print("Setup complete");
}

void loop()
{
    SWval = SWread();
    SWprint(toupper(SWval));
}

