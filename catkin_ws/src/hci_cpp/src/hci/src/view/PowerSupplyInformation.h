//
// Created by david on 18/06/17.
//

#ifndef HCI_CPP_POWERSUPPLYINFORMATION_H
#define HCI_CPP_POWERSUPPLYINFORMATION_H


#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>

class PowerSupplyInformation : public QWidget{

public:
    PowerSupplyInformation(QWidget* parent=0);
    virtual  ~PowerSupplyInformation(){};

public slots:
    void setInputVoltage(double value);
    void setInputCurrent(double value);
    void setOutputVoltage(double value);
    void setOutputCurrent(double value);
    void setOutputPower(double value);
    void setTemperature(double value);

private:
    QLabel* pInputVoltage;
    QLabel* pInputCurrent;
    QLabel* pOutputVoltage;
    QLabel* pOutputCurrent;
    QLabel* pOutputPower;
    QLabel* pTemperature;
};


#endif //HCI_CPP_POWERSUPPLYINFORMATION_H
