//
// Created by david on 18/06/17.
//

#ifndef HCI_CPP_MAINVIEW_H
#define HCI_CPP_MAINVIEW_H


#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <model/DriveData.h>
#include "JoystickView.h"
#include "AttitudeView.h"
#include "PowerSupplyInformation.h"
#include "DriveView.h"
#include "ArmView.h"

class MainView : public QWidget{

    Q_OBJECT

public:
    MainView(QWidget* parent=0);
    virtual ~MainView() {};

    JoystickView* getJoystickView();;

public slots:
    void updateSteeringMode(const SteeringMode& mode);
    void updateDriveStatus(const DriveStatusData& statusData);
    void setInputVoltage(double value);
    void setInputCurrent(double value);
    void setOutputVoltage(double value);
    void setOutputCurrent(double value);
    void setOutputPower(double value);
    void setTemperature(double value);
    void setMotorEnable(bool enable);

signals:
    void steeringModeChanged(const SteeringMode& mode);
    void joystickModeChanged(QString modeText);

private:
    void addLine(QVBoxLayout* layout);
    JoystickView* joystickView;
    AttitudeView* attitudeView;
    PowerSupplyInformation* powerSupplyInformation;
    DriveView* driveView;
    QLabel* pMotorEnableStatus;
    ArmView* armView;

};


#endif //HCI_CPP_MAINVIEW_H
