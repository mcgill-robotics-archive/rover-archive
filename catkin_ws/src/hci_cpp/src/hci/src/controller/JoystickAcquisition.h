//
// Created by David Lavoie-Boutin on 20/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_JOYSTICKACQUISITION_H
#define HCI_CPP_JOYSTICKACQUISITION_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QSlider>
#include <QtWidgets/QPushButton>
#include <model/JoystickData.h>

/**
 * @brief Temporary class stub to show POC for joystick acquisition and controller handling
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
    void joystickDataUpdated(JoystickData data);

private:
    QSlider* a1;
    QSlider* a2;
    QSlider* a3;
    QSlider* a4;

    QPushButton* b1;
    QPushButton* b2;
    QPushButton* b3;
    QPushButton* b4;
    QPushButton* b5;
    QPushButton* b6;
    QPushButton* b7;
    QPushButton* b8;
    QPushButton* b9;
    QPushButton* b10;
    QPushButton* b11;
    QPushButton* b12;

private slots:
    void updateCommand();
};


#endif //HCI_CPP_JOYSTICKACQUISITION_H
