//
// Created by david on 18/06/17.
//

#ifndef HCI_CPP_ATTITUDEVIEW_H
#define HCI_CPP_ATTITUDEVIEW_H


#include <QtWidgets/QWidget>

#include <qFlightInstruments/QAttitude.h>
#include <qFlightInstruments/QCompass.h>
#include "PoseDisplay.h"

class AttitudeView : public QWidget {

    Q_OBJECT

public:
    AttitudeView(QWidget* parent = NULL);
    virtual ~AttitudeView() {};

public slots:
    void setPitch(float value);
    void setRoll(float value);
    void setYaw(float value);
    void setLongitude(float value);
    void setLatitude(float value);

private:
    QADI* pAdi;
    QCompass* pCompass;
    PoseDisplay* pPoseDisplay;
};


#endif //HCI_CPP_ATTITUDEVIEW_H
