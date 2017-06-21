//
// Created by David Lavoie-Boutin on 20/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_JOYSTICKINTERFACE_H
#define HCI_CPP_JOYSTICKINTERFACE_H


#include <model/JoystickData.h>

/**
 * @brief Abstract interface class must implement to be eligible to receive
 * joystick input
 *
 * A class implementing this interface can be registered with a JoystickController
 */
class JoystickInterface : public QObject {
    Q_OBJECT

public slots:
    /**
     * @brief Function receiving the joystick data when the controller is active
     *
     * The JoystickController will connect this instance to the joystick acquisition
     * usgin this function as the slot for the signal with the joystick data.
     *
     * @param data The newest joystick data
     */
    virtual void handleJoystickData(JoystickData& data) = 0;
};

#endif //HCI_CPP_JOYSTICKINTERFACE_H
