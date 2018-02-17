//
// Created by David Lavoie-Boutin on 2016-02-02.
//

#include "ram.h"

int RAM::freeRam ()
{
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void RAM::freeRamCallback(const arduino_msgs::Ram::Request & request, arduino_msgs::Ram::Response & response)
{
    response.ram = freeRam();
}

