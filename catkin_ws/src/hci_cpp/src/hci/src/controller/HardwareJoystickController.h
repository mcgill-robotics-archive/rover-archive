//
// Created by David Lavoie-Boutin on 01/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_HARDWAREJOYSTICKCONTROLLER_H
#define HCI_CPP_HARDWAREJOYSTICKCONTROLLER_H

#include <QObject>
#include <model/JoystickData.h>
#include "joystick.h"

/**
 * @brief Wrapper for the Joystick driver. 
 * 
 * Class offers the process() function that starts a loop to periodically 
 * populate the JoystickData structure. When the structure is updated, the 
 * signal joystickDataUpdated is emmited with a copy of the new data.
 */
class HardwareJoystickController : public QObject {
Q_OBJECT
public:
    /**
     * @brief Attempts to find the joystick to the hard-coded path 
     * `/dev/input/js0`
     * 
     * Only one controller is supported at the moment, the controller present
     * at file pointer above (if it exists) will be used for the entire time 
     * the application runs. 
     * 
     * Dynamic plug-and-play is not supported.
     */ 
    HardwareJoystickController();

    virtual ~HardwareJoystickController() {};

    /**
     * @brief Can tell wether the driver has found and initialized the 
     * joystick properly and is ready for use.
     * 
     * @return True if the joystick is ready for use.
     */
    bool joystickFound();

public slots:
    /**
     * @brief Start the infinite loop to read and update the joystick values.
     * 
     * Just like all other controllers, this function should never return. It 
     * is meant to run in a dedicated thread. 
     * 
     * The joystickDataUpdated signal 
     * is emmited at every iteration.  
     */
    void process();

signals:
    void joystickDataUpdated(JoystickData data);

private:

    Joystick mJoystick;
    JoystickData mData;
};


#endif //HCI_CPP_HARDWAREJOYSTICKCONTROLLER_H
