//
// Created by Vanessa Roe on 10/03/18.
// vanessa.roe (at) mail.mcgill.ca
//
//called in: SingleCameraView.h

#ifndef HCI_CPP_CAMERAANGLESELECT_H
#define HCI_CPP_CAMERAANGLESELECT_H

#include <QtWidgets/QRadioButton>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QWidget>
#include <QChar>
#include <ros/ros.h>


class CameraAngleSelect : public QWidget {

Q_OBJECT

public:
    CameraAngleSelect (QWidget *parent= nullptr, int start_angle=0);
    void defaultAngle(int ang);
    void changeAngle();
    QString angleText(int angle);

signals:
    

private:
    QHBoxLayout layout;
    QRadioButton* deg0;
    QRadioButton* deg90;
    QRadioButton* deg180;
    QRadioButton* deg270;
    int angle;
};

#endif //HCI_CPP_CAMERAANGLESELECT_H
