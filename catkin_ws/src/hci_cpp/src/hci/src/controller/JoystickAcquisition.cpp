//
// Created by David Lavoie-Boutin on 20/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <QtWidgets/QSlider>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QPushButton>
#include "JoystickAcquisition.h"
#include <QDebug>

JoystickAcquisition::JoystickAcquisition(QWidget *parent) : QWidget(parent) {
    //TODO: Auto-Generated Constructor Stub

    a1 = new QSlider(Qt::Vertical, this);
    a2 = new QSlider(Qt::Vertical, this);
    a3 = new QSlider(Qt::Vertical, this);
    a4 = new QSlider(Qt::Vertical, this);
    a1->setRange(-100, 100);
    a2->setRange(-100, 100);
    a3->setRange(-100, 100);
    a4->setRange(-100, 100);

    QHBoxLayout* hbox = new QHBoxLayout;
    hbox->addWidget(a1);
    hbox->addWidget(a2);
    hbox->addWidget(a3);
    hbox->addWidget(a4);

    QGridLayout* grid = new QGridLayout;

    b1 = new QPushButton("B1, Enable", this);
    b2 = new QPushButton("B2, Ackerman", this);
    b3 = new QPushButton("B3, Point", this);
    b4 = new QPushButton("B4, Translate", this);
    b5 = new QPushButton("B5", this);
    b6 = new QPushButton("B6", this);
    b7 = new QPushButton("B7, Open Loop", this);
    b8 = new QPushButton("B8, Closed Loop", this);
    b9 = new QPushButton("B9", this);
    b10 = new QPushButton("B10", this);
    b11 = new QPushButton("B11", this);
    b12 = new QPushButton("B12", this);

    grid->addWidget(b1, 0, 0);
    grid->addWidget(b2, 0, 1);
    grid->addWidget(b3, 0, 2);
    grid->addWidget(b4, 0, 3);
    grid->addWidget(b5, 1, 0);
    grid->addWidget(b6, 1, 1);
    grid->addWidget(b7, 1, 2);
    grid->addWidget(b8, 1, 3);
    grid->addWidget(b9, 2, 0);
    grid->addWidget(b10, 2, 1);
    grid->addWidget(b11, 2, 2);
    grid->addWidget(b12, 2, 3);

    hbox->addItem(grid);
    setLayout(hbox);

    connect(a1, &QSlider::valueChanged, this, &JoystickAcquisition::updateCommand);
    connect(a2, &QSlider::valueChanged, this, &JoystickAcquisition::updateCommand);
    connect(a3, &QSlider::valueChanged, this, &JoystickAcquisition::updateCommand);
    connect(a4, &QSlider::valueChanged, this, &JoystickAcquisition::updateCommand);
    connect(b1, &QPushButton::pressed, this, &JoystickAcquisition::updateCommand);
    connect(b2, &QPushButton::pressed, this, &JoystickAcquisition::updateCommand);
    connect(b3, &QPushButton::pressed, this, &JoystickAcquisition::updateCommand);
    connect(b4, &QPushButton::pressed, this, &JoystickAcquisition::updateCommand);
    connect(b5, &QPushButton::pressed, this, &JoystickAcquisition::updateCommand);
    connect(b6, &QPushButton::pressed, this, &JoystickAcquisition::updateCommand);
    connect(b7, &QPushButton::pressed, this, &JoystickAcquisition::updateCommand);
    connect(b8, &QPushButton::pressed, this, &JoystickAcquisition::updateCommand);
    connect(b9, &QPushButton::pressed, this, &JoystickAcquisition::updateCommand);
    connect(b10, &QPushButton::pressed, this, &JoystickAcquisition::updateCommand);
    connect(b11, &QPushButton::pressed, this, &JoystickAcquisition::updateCommand);
    connect(b12, &QPushButton::pressed, this, &JoystickAcquisition::updateCommand);
    connect(b1, &QPushButton::released, this, &JoystickAcquisition::updateCommand);
    connect(b2, &QPushButton::released, this, &JoystickAcquisition::updateCommand);
    connect(b3, &QPushButton::released, this, &JoystickAcquisition::updateCommand);
    connect(b4, &QPushButton::released, this, &JoystickAcquisition::updateCommand);
    connect(b5, &QPushButton::released, this, &JoystickAcquisition::updateCommand);
    connect(b6, &QPushButton::released, this, &JoystickAcquisition::updateCommand);
    connect(b7, &QPushButton::released, this, &JoystickAcquisition::updateCommand);
    connect(b8, &QPushButton::released, this, &JoystickAcquisition::updateCommand);
    connect(b9, &QPushButton::released, this, &JoystickAcquisition::updateCommand);
    connect(b10, &QPushButton::released, this, &JoystickAcquisition::updateCommand);
    connect(b11, &QPushButton::released, this, &JoystickAcquisition::updateCommand);
    connect(b12, &QPushButton::released, this, &JoystickAcquisition::updateCommand);
}

void JoystickAcquisition::updateCommand() {
    JoystickData data;
    data.buttons.resize(12);

    data.a1 = a1->value() / 100.0;
    data.a2 = a2->value() / 100.0;
    data.a3 = a3->value() / 100.0;
    data.a4 = a4->value() / 100.0;

    data.buttons[0] = b1->isDown();
    data.buttons[1] = b2->isDown();
    data.buttons[2] = b3->isDown();
    data.buttons[3] = b4->isDown();
    data.buttons[4] = b5->isDown();
    data.buttons[5] = b6->isDown();
    data.buttons[6] = b7->isDown();
    data.buttons[7] = b8->isDown();
    data.buttons[8] = b9->isDown();
    data.buttons[9] = b10->isDown();
    data.buttons[10] = b11->isDown();
    data.buttons[11] = b12->isDown();

    emit joystickDataUpdated(data);
//    qDebug() << "Sent new joystick data";
}
