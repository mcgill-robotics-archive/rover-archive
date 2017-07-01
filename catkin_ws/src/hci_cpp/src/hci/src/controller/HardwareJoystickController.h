//
// Created by David Lavoie-Boutin on 01/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_HARDWAREJOYSTICKCONTROLLER_H
#define HCI_CPP_HARDWAREJOYSTICKCONTROLLER_H

#include <QObject>
#include <model/JoystickData.h>
#include "joystick.h"


class HardwareJoystickController : public QObject {
Q_OBJECT
public:
    HardwareJoystickController();

    virtual ~HardwareJoystickController() {};
    bool joystickFound();

public slots:
    void process();

signals:
    void joystickDataUpdated(JoystickData data);

private:

    Joystick mJoystick;
    JoystickData mData;
};


#endif //HCI_CPP_HARDWAREJOYSTICKCONTROLLER_H
