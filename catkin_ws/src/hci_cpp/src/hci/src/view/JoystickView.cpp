//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <QtWidgets/QPushButton>
#include "JoystickView.h"

JoystickView::JoystickView(QWidget *parent) : QWidget(parent), mJoystickControllerCount(0) {
    pLayout = new QGridLayout;
    pButtonGroup = new QButtonGroup(this);
    pButtonGroup->setExclusive(true);
    setLayout(pLayout);

    // Connect button group to anonymous function to emit the button text
    // Signal QButtonGroup::buttonClicked will be emitted any time a button
    // in the group is pressed, so we don't need to know the buttons when
    // creating the connection and the slot.
    connect(pButtonGroup, static_cast<void(QButtonGroup::*)(QAbstractButton *)>(&QButtonGroup::buttonClicked),
            this, [=](QAbstractButton *button) {emit modeChanged(button->text());});
}

void JoystickView::addMode(QString name) {
    // Create new button
    QPushButton* modeButton = new QPushButton(name, this);
    modeButton->setCheckable(true);

    // Add button to layout and button group to get exclusive behaviour
    pLayout->addWidget(modeButton, mJoystickControllerCount / 2, mJoystickControllerCount % 2);
    pButtonGroup->addButton(modeButton);

    mJoystickControllerCount++;
}
