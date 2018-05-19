//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <QtWidgets/QWidget>
#include <QtWidgets/QGridLayout>
#include "ScienceView.h"

ScienceView::ScienceView(QWidget *parent) : QWidget(parent) {
    QGridLayout* grid  = new QGridLayout;
    QGridLayout* grid2  = new QGridLayout;

    QLabel* windSpeedTitle = new QLabel("Wind Speed", this);
    windSpeed = new QLabel(this);
    grid2->addWidget(windSpeedTitle, 0, 0);
    grid2->addWidget(windSpeed, 0, 1);

    QLabel* humidityTitle = new QLabel("Humidity", this);
    humidity = new QLabel(this);

    grid2->addWidget(humidityTitle, 0, 2);
    grid2->addWidget(humidity, 0, 3);

    QLabel* temperatureTitle = new QLabel("Temperature", this);
    temperature = new QLabel(this);
    grid2->addWidget(temperatureTitle, 0, 4);
    grid2->addWidget(temperature, 0, 5);

    QLabel* carriageSpeedTitle = new QLabel("Carriage Speed", this);
    carriageSpeed = new QLabel(this);
    grid->addWidget(carriageSpeedTitle, 0, 0);
    grid->addWidget(carriageSpeed, 0, 1);

    QLabel* carriagePositionTitle = new QLabel("Carriage Position", this);
    carriagePosition = new QLabel(this);
    grid->addWidget(carriagePositionTitle, 1, 0);
    grid->addWidget(carriagePosition, 1, 1);

    QLabel* drillSpeedTitle = new QLabel("Drill Speed", this);
    drillSpeed = new QLabel(this);
    grid->addWidget(drillSpeedTitle, 0, 2);
    grid->addWidget(drillSpeed, 0, 3);

    QLabel* drillPositionTitle = new QLabel("Drill Position", this);
    drillPosition = new QLabel(this);
    grid->addWidget(drillPositionTitle, 1, 2);
    grid->addWidget(drillPosition, 1, 3);

    QLabel* probeSpeedTitle = new QLabel("Probe Speed", this);
    probeSpeed = new QLabel(this);
    grid->addWidget(probeSpeedTitle, 2, 0);
    grid->addWidget(probeSpeed, 2, 1);

    QVBoxLayout* mainBox = new QVBoxLayout();
    QLabel* motorTitle = new QLabel("Sampling Status", this);
    QLabel* sensorTitle = new QLabel("Sensor Values", this);

    QFont titleFont;
    titleFont.setPointSize(15);
    titleFont.setBold(true);
    titleFont.setWeight(75);
    motorTitle->setFont(titleFont);
    sensorTitle->setFont(titleFont);

    mainBox->addWidget(motorTitle);
    mainBox->addItem(grid);
    mainBox->addWidget(sensorTitle);
    mainBox->addItem(grid2);

    setLayout(mainBox);
}

//update the probe speed displayed --> DOES NOT connect to actal probe (pretty sure)
void ScienceView::setProbeSpeed(float value) {
    QString str;
    str.setNum(value,'f',2);
    str.append(""); //add text here to indicate the units of the speed
    probeSpeed->setText(str);
}

//update the drill speed displayed --> DOES NOT connect to actal drill (pretty sure)
void ScienceView::setDrillSpeed(float value){
    QString str;
    str.setNum(value,'f',2);
    str.append(""); //add text here to indicate the units of the speed
    drillSpeed->setText(str);
}

//update the drill speed displayed --> DOES NOT connect to actal drill (pretty sure)
void ScienceView::setCarriageSpeed(float value){
    QString str;
    str.setNum(value,'f',2);
    str.append(""); //add text here to indicate the units of the speed
    carriageSpeed->setText(str);
}

