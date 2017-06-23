//
// Created by David Lavoie-Boutin on 23/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_ARMCONTROLLER_H
#define HCI_CPP_ARMCONTROLLER_H

#include <QtWidgets/QWidget>
#include "JoystickInterface.h"


class ArmController : public JoystickInterface {
Q_OBJECT
public:
    ArmController();

    virtual ~ArmController() {};

    virtual void handleJoystickData(JoystickData data);
};


#endif //HCI_CPP_ARMCONTROLLER_H
