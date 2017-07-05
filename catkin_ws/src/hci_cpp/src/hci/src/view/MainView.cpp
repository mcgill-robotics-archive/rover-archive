//
// Created by david on 18/06/17.
//

#include <ros/ros.h>
#include "MainView.h"
#include "utilities.h"

MainView::MainView(QWidget *parent) : QWidget(parent) {
    QVBoxLayout * vbox = new QVBoxLayout;
    QHBoxLayout * hbox = new QHBoxLayout;

    // Motor Enable information:
    {
        QLabel *enableTitle = new QLabel("Motor Condition:", this);
        pMotorEnableStatus = new QLabel(this);
        pMotorEnableStatus->setAlignment(Qt::AlignCenter);
        QHBoxLayout *hbox = new QHBoxLayout;
        hbox->addWidget(enableTitle);
        hbox->addWidget(pMotorEnableStatus);
        vbox->addItem(hbox);
    }

    attitudeView = new AttitudeView(this);
    powerSupplyInformation = new PowerSupplyInformation(this);
    driveView = new DriveView(this);
    joystickView = new JoystickView(this);
    armView = new ArmView(this);
    navCameraView = new NavCameraView(this);

    vbox->addWidget(driveView);
    addLine(vbox);
    vbox->addWidget(joystickView);
    addLine(vbox);
    vbox->addWidget(armView);
    addLine(vbox);
    vbox->addWidget(attitudeView);
    addLine(vbox);
    vbox->addWidget(powerSupplyInformation);

    hbox->addWidget(navCameraView);
    hbox->addItem(vbox);
    setLayout(hbox);

    connect(driveView, &DriveView::steeringModeChanged, this, &MainView::steeringModeChanged);
    connect(driveView, &DriveView::steeringModeChanged, this, &MainView::updateSteeringMode);
    connect(joystickView, &JoystickView::modeChanged, this, &MainView::joystickModeChanged);
    connect(armView, &ArmView::closeLoopModeChanged, this, &MainView::closeLoopModeChanged);
    connect(armView, &ArmView::armJointChanged, this, &MainView::armJointChanged);
    connect(armView, &ArmView::armModeChanged, this, &MainView::armModeChanged);
}

void MainView::addLine(QVBoxLayout *layout) {
    QFrame* line_1 = new QFrame(this);
    line_1->setFrameShape(QFrame::HLine);
    line_1->setFrameShadow(QFrame::Sunken);
    layout->addWidget(line_1);
}

void MainView::updateSteeringMode(const SteeringMode &mode) {
    driveView->updateSteeringMode(mode);
}

void MainView::setMotorEnable(bool enable) {
    if (enable) {
        setOkBG(pMotorEnableStatus, "Enabled");
    } else {
        setBadBG(pMotorEnableStatus, "Disabled");
    }
}

JoystickView *MainView::getJoystickView()
{
    return joystickView;
}

void MainView::updateDriveStatus(const DriveStatusData &statusData) {
    driveView->updateDriveStatus(statusData);
}

void MainView::setInputVoltage(double value) {
    powerSupplyInformation->setInputVoltage(value);
}

void MainView::setInputCurrent(double value) {
    powerSupplyInformation->setInputCurrent(value);
}

void MainView::setOutputVoltage(double value) {
    powerSupplyInformation->setOutputVoltage(value);
}

void MainView::setOutputCurrent(double value) {
    powerSupplyInformation->setOutputCurrent(value);
}

void MainView::setOutputPower(double value) {
    powerSupplyInformation->setOutputPower(value);
}

void MainView::setTemperature(double value) {
    powerSupplyInformation->setTemperature(value);
}

void MainView::setArmMode(ArmMode mode) {
    armView->setArmMode(mode);
}

void MainView::changeArmJoint(ArmJoint joint) {
    armView->changeArmJoint(joint);
}

void MainView::changeCloseLoopMode(ArmClosedLoopMode mode) {
    armView->changeCloseLoopMode(mode);
}

NavCameraView *MainView::getNavCamView() {
    return navCameraView;
}
