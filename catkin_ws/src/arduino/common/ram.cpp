//
// Created by David Lavoie-Boutin on 2016-02-02.
//

#include "ram.h"

int freeRam ()
{
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void freeRamCallback(const arduino::ram::Request & request, arduino::ram::Response & response)
{
    response.ram = freeRam();
}

