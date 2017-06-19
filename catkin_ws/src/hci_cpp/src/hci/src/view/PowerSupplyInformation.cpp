//
// Created by david on 18/06/17.
//

#include <QtWidgets/QVBoxLayout>
#include "PowerSupplyInformation.h"

PowerSupplyInformation::PowerSupplyInformation(QWidget *parent) : QWidget(parent)
{
    QFont titleFont;
    titleFont.setPointSize(15);
    titleFont.setBold(true);
    titleFont.setWeight(75);

    QVBoxLayout* v1 = new QVBoxLayout();

    QLabel* title = new QLabel("Power Supply Information", this);
    title->setFont(titleFont);

    pInputVoltage = new QLabel(this);
    QLabel* inputVoltage = new QLabel("Input Voltage: ", this);
    pInputCurrent = new QLabel(this);
    QLabel* inputCurrent = new QLabel("Input Current: ", this);
    pOutputVoltage = new QLabel(this);
    QLabel* outputVoltage = new QLabel("Output Voltage: ", this);
    pOutputCurrent = new QLabel(this);
    QLabel* outputCurrent = new QLabel("Output Current: ", this);
    pOutputPower = new QLabel(this);
    QLabel* outputPower = new QLabel("Output Power: ", this);
    pTemperature = new QLabel(this);
    QLabel* temperature = new QLabel("Temperature: ", this);

    QGridLayout* grid = new QGridLayout();
    grid->addWidget(inputVoltage, 0, 0);
    grid->addWidget(inputCurrent, 1, 0);
    grid->addWidget(temperature, 2, 0);
    grid->addWidget(outputVoltage, 0, 2);
    grid->addWidget(outputCurrent, 1, 2);
    grid->addWidget(outputPower, 2, 2);

    grid->addWidget(pInputVoltage, 0, 1);
    grid->addWidget(pInputCurrent, 1, 1);
    grid->addWidget(pTemperature, 2, 1);
    grid->addWidget(pOutputVoltage, 0, 3);
    grid->addWidget(pOutputCurrent, 1, 3);
    grid->addWidget(pOutputPower, 2, 3);

    v1->addWidget(title);
    v1->addItem(grid);
    this->setLayout(v1);
    this->setMaximumHeight(100);

}

void PowerSupplyInformation::setInputVoltage(double value) {
    QString str;
    str.setNum(value, 'f', 2);
    str.append(" V");
    pInputVoltage->setText(str);
}

void PowerSupplyInformation::setInputCurrent(double value) {
    QString str;
    str.setNum(value, 'f', 2);
    str.append(" A");
    pInputCurrent->setText(str);
}

void PowerSupplyInformation::setOutputVoltage(double value) {
    QString str;
    str.setNum(value, 'f', 2);
    str.append(" V");
    pOutputVoltage->setText(str);
}

void PowerSupplyInformation::setOutputCurrent(double value) {
    QString str;
    str.setNum(value, 'f', 2);
    str.append(" A");
    pOutputCurrent->setText(str);
}

void PowerSupplyInformation::setOutputPower(double value) {
    QString str;
    str.setNum(value, 'f', 2);
    str.append(" W");
    pOutputPower->setText(str);
}

void PowerSupplyInformation::setTemperature(double value) {
    QString str;
    str.setNum(value, 'f', 2);
    str.append(" ");
    str.append(QChar(176));
    pTemperature->setText(str);
}
