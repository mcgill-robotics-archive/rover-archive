//
// Created by David Lavoie-Boutin on 2015-10-15.
//

//Created August 23 2006
//Heather Dewey-Hagborg
//http://www.arduino.cc

#include <ctype.h>
#include <Arduino.h>
#include "main.h"

#define bit9600Delay 100
#define halfBit9600Delay 50
#define bit4800Delay 188
#define halfBit4800Delay 94

byte rx = 6;
byte tx = 7;
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

void SWprint(int data)
{
    byte mask;
    //startbit
    digitalWrite(tx,LOW);
    delayMicroseconds(bit9600Delay);
    for (mask = 0x01; mask>0; mask <<= 1) {
        if (data & mask){ // choose bit
            digitalWrite(tx,HIGH); // send 1
        }
        else{
            digitalWrite(tx,LOW); // send 0
        }
        delayMicroseconds(bit9600Delay);
    }
    //stop bit
    digitalWrite(tx, HIGH);
    delayMicroseconds(bit9600Delay);
}

byte SWread()
{
    Serial.print("Entering serial read");
    byte val = 0;
    while (digitalRead(rx));
    Serial.print("past the while");
    //wait for start bit
    if (digitalRead(rx) == LOW) {
        delayMicroseconds(halfBit9600Delay);
        for (int offset = 0; offset < 8; offset++) {
            delayMicroseconds(bit9600Delay);
            val |= digitalRead(rx) << offset;
        }
        //wait for stop bit + extra
        delayMicroseconds(bit9600Delay);
        delayMicroseconds(bit9600Delay);
        return val;
    }
}
