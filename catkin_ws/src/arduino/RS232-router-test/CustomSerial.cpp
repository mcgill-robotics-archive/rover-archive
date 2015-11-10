//
// Created by david on 11/10/15.
//

#include <Arduino.h>
#include "CustomSerial.h"


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
    return 0;
}
