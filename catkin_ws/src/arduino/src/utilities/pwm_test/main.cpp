//
// Created by david on 5/15/16.
//


#include <Servo.h>
#include <HardwareSerial.h>
#include <Arduino.h>

Servo esc;

void setup() {
    // Open serial communications and wait for port to open:
    Serial.begin(115200);
    esc.attach(7);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    esc.writeMicroseconds(1500);
}

void loop() { // run over and over
    if (Serial.available()) {
        esc.writeMicroseconds((int) Serial.parseInt());
//        Serial.println(serial_int);
    }
    delay(10);
}

