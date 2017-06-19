//
// Created by david on 18/06/17.
//

#include <QtWidgets/QHBoxLayout>
#include "AttitudeView.h"

AttitudeView::AttitudeView(QWidget *parent) {
    QHBoxLayout * hbox = new QHBoxLayout;
    QVBoxLayout * vbox = new QVBoxLayout;

    pAdi = new QADI(this);
    pCompass = new QCompass(this);
    pPoseDisplay = new PoseDisplay(this);

    hbox->addWidget(pAdi);
    hbox->addWidget(pCompass);
    vbox->addItem(hbox);
    vbox->addWidget(pPoseDisplay);

    this->setLayout(vbox);
}

void AttitudeView::setPitch(float value) {
    pAdi->setPitch(value);
    pPoseDisplay->setPitch(value);
}

void AttitudeView::setRoll(float value) {
    pAdi->setRoll(value);
    pPoseDisplay->setRoll(value);
}

void AttitudeView::setYaw(float value) {
    pCompass->setYaw(value);
    pPoseDisplay->setYaw(value);
}

void AttitudeView::setLongitude(float value) {
    pPoseDisplay->setLongitude(value);

}

void AttitudeView::setLatitude(float value) {
    pPoseDisplay->setLatitude(value);

}
