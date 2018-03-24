//
// Created by Vanessa Roe on 10/03/18.
// vanessa.roe (at) mail.mcgill.ca
//

#include <QtWidgets/QHBoxLayout>
#include "CameraAngleSelect.h"

CameraAngleSelect::CameraAngleSelect(QWidget *parent, int start_angle) : QWidget(parent) {
    angle = start_angle;
   
    QHBoxLayout *layout = new QHBoxLayout;
    deg0 = new QRadioButton;
    deg90 = new QRadioButton;
    deg180 = new QRadioButton;
    deg270 = new QRadioButton;

    layout->addWidget(deg0);
    layout->addWidget(deg90);
    layout->addWidget(deg180);
    layout->addWidget(deg270);

    deg0->setText(angleText(0));
    deg90->setText(angleText(90));
    deg180->setText(angleText(180));
    deg270->setText(angleText(270));

    setLayout(layout);

    deg0->click();
    
    connect(deg0, &QRadioButton::clicked, this, &CameraAngleSelect::changeAngle);
    connect(deg90, &QRadioButton::clicked, this, &CameraAngleSelect::changeAngle);
    connect(deg180, &QRadioButton::clicked, this, &CameraAngleSelect::changeAngle);
    connect(deg270, &QRadioButton::clicked, this, &CameraAngleSelect::changeAngle);
 
//      self._deg0.toggled.connect(self._emit_signal)
 
     defaultAngle(angle);
}

void CameraAngleSelect::defaultAngle(int ang){
    if(ang == 90)
         deg90->click();
    else if (ang == 180)
        deg180->click();
    else if (ang == 270)
        deg270->click();
    else
        deg0->click();
}

void CameraAngleSelect::changeAngle(){
    if(deg0->isChecked()){
        angle = 90;
        ROS_INFO("CameraAngleSelect: angle is changed to 90");
    }
    else if (deg180->isChecked()){
        angle = 180;   
        ROS_INFO("CameraAngleSelect: angle is changed to 180");
    }
    else if (deg270->isChecked()){
        angle = 270;
        ROS_INFO("CameraAngleSelect: angle is changed to 270");
    }
    else{
        angle = 0; 
        ROS_INFO("CameraAngleSelect: angle is changed to 0");
    }
}

QString CameraAngleSelect::angleText(int angle){
    QString str;
    if (angle == 90)
        str = "90";
    else if (angle == 180)
        str = "180";
    else if (angle == 270)
        str = "270";
    else
        str = "0";
    str.append(QChar(176));
    return str;
}









