//
// Created by david on 18/06/17.
//

#ifndef HCI_CPP_POSEDISPLAY_H
#define HCI_CPP_POSEDISPLAY_H


#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QCheckBox>

class PoseDisplay : public QWidget {
    Q_OBJECT

public:
    PoseDisplay(QWidget* parent=NULL);
    virtual ~PoseDisplay() {};

public slots:
    void setPitch(float value);
    void setRoll(float value);
    void setYaw(float value);
    void setLongitude(float value);
    void setLatitude(float value);

private:
    QLabel* pLatitude;
    QLabel* pLongitude;
    QLabel* pPitch;
    QLabel* pRoll;
    QLabel* pYaw;
};


#endif //HCI_CPP_POSEDISPLAY_H
