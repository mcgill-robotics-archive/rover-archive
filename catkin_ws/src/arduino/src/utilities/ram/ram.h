//
// Created by David Lavoie-Boutin on 2016-02-08.
//

#ifndef ROVER_ARDUINO_RAM_H
#define ROVER_ARDUINO_RAM_H

#include <arduino_msgs/Ram.h>


class RAM {
public:
    static int freeRam ();
    static void freeRamCallback(const arduino_msgs::Ram::Request & request, arduino_msgs::Ram::Response & response);
};
#endif //ROVER_ARDUINO_RAM_H
