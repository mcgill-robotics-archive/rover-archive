//
// Created by David Lavoie-Boutin on 20/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_JOYSTICKACQUISITION_H
#define HCI_CPP_JOYSTICKACQUISITION_H

#include <QtWidgets/QWidget>
#include <model/JoystickData.h>

/**
 * !@brief Temporary class stub to show POC for joystick acquisition and controller handling
 *
 * This class is responsible for interfacing with the hardware joystick and
 * acquire the axis and button data. When new data is acquired, emit the
 * member signal with the new data
 *
 * TODO: implement real acquisition class
 */
class JoystickAcquisition : public QWidget {
Q_OBJECT
public:
    JoystickAcquisition(QWidget *parent = 0);

    virtual ~JoystickAcquisition() {};
signals:
    void joystickDataUpdated(JoystickData& data);
};


#endif //HCI_CPP_JOYSTICKACQUISITION_H
