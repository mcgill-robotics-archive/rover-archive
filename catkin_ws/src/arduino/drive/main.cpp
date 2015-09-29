#include <Arduino.h>

#include <Servo.h>


Servo servo;

void setup()
{
    servo.attach(10);
}

int i = 0;

void loop()
{
    i ++;
    delay(10);
}
