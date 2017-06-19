//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_JOYSTICKDATA_H
#define HCI_CPP_JOYSTICKDATA_H

#include <QBitArray>

struct JoystickData
{
    float a1;
    float a2;
    float a3;
    float a4;
    QBitArray buttons;
};

#endif //HCI_CPP_JOYSTICKDATA_H
