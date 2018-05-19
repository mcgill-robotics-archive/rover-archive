//
// Created by david on 18/06/17.
//

#ifndef HCI_CPP_MAINVIEW_H
#define HCI_CPP_MAINVIEW_H


#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <std_msgs/Float32.h>
#include <model/DriveData.h>
#include "JoystickView.h"
#include "AttitudeView.h"
#include "PowerSupplyInformation.h"
#include "DriveView.h"
#include "ArmView.h"
#include "NavCameraView.h"
#include "ScienceView.h"
#include "CameraFilterView.h"

class MainView : public QWidget{

    Q_OBJECT

public:
    MainView(QWidget* parent=0);
    virtual ~MainView() {};

    JoystickView* getJoystickView();
    NavCameraView* getNavCamView();

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
    void setArmMode(ArmMode mode);
    void changeArmJoint(ArmJoint joint);
    void changeCloseLoopMode(ArmClosedLoopMode mode);
    void setPitch(float value);
    void setRoll(float value);
    void setYaw(float value);
    void setLongitude(float value);
    void setLatitude(float value);
    void setProbeSpeed(float value);
    void setDrillSpeed(float value);
    void setCarriageSpeed(float value);

signals:
    void steeringModeChanged(const SteeringMode& mode);
    void joystickModeChanged(QString modeText);
    void closeLoopModeChanged(ArmClosedLoopMode mode);
    void armJointChanged(ArmJoint joint);
    void armModeChanged(ArmMode mode);

private:
    void addLine(QVBoxLayout* layout);
    JoystickView* joystickView;
    AttitudeView* attitudeView;
    PowerSupplyInformation* powerSupplyInformation;
    DriveView* driveView;
    QLabel* pMotorEnableStatus;
    ArmView* armView;
    NavCameraView* navCameraView;
    ScienceView* scienceView;
    CameraFilterView* cameraFilterView;

};


#endif //HCI_CPP_MAINVIEW_H
