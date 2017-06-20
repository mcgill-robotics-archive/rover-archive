//
// Created by David Lavoie-Boutin on 20/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_JOYSTICKINTERFACE_H
#define HCI_CPP_JOYSTICKINTERFACE_H


#include <model/JoystickData.h>

class JoystickInterface {

public:
    virtual void handleJoystickData(JoystickData& data) = 0;
};


#endif //HCI_CPP_JOYSTICKINTERFACE_H
