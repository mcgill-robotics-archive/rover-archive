//
// Created by david on 18/06/17.
//

#include "MainView.h"

MainView::MainView(QWidget *parent) : QWidget(parent) {
    QVBoxLayout * vbox = new QVBoxLayout;

    attitudeView = new AttitudeView(this);
    powerSupplyInformation = new PowerSupplyInformation(this);
    driveView = new DriveView(this);
    joystickView = new JoystickView(this);

    vbox->addWidget(driveView);
    addLine(vbox);
    vbox->addWidget(joystickView);
    addLine(vbox);
    vbox->addWidget(attitudeView);
    addLine(vbox);
    vbox->addWidget(powerSupplyInformation);
    setLayout(vbox);

    connect(driveView, &DriveView::steeringModeChanged, this, &MainView::steeringModeChanged);
    connect(driveView, &DriveView::steeringModeChanged, this, &MainView::updateSteeringMode);
    connect(joystickView, &JoystickView::modeChanged, this, &MainView::joystickModeChanged);

    addJoystickMode("test mode");
    addJoystickMode("test mode1");
}

void MainView::addLine(QVBoxLayout *layout) {
    QFrame* line_1 = new QFrame(this);
    line_1->setFrameShape(QFrame::HLine);
    line_1->setFrameShadow(QFrame::Sunken);
    layout->addWidget(line_1);
}

void MainView::updateSteeringMode(const SteeringMode &mode) {
    // TODO: Delete at some point, this is useless, but fun
    QString modeText;
    if (mode == ACKERMANN) modeText = QString("ACKERMAN");
    if (mode == POINT) modeText = QString("POINT");
    if (mode == TRANSLATE) modeText = QString("TRANSLATE");
    qDebug() << "MainView.cpp: " << modeText;
}

void MainView::addJoystickMode(QString modeTitle) {
    joystickView->addMode(modeTitle);
}
