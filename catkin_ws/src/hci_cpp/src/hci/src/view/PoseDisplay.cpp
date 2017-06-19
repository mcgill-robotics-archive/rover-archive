//
// Created by david on 18/06/17.
//

#include <QtWidgets/QHBoxLayout>
#include <QDebug>
#include "PoseDisplay.h"

QString displayAngle(float angle)
{
    QString str;

    str.setNum(angle, 'f', 3);
    str.append(QChar(176));
    return str;
}

PoseDisplay::PoseDisplay(QWidget *parent) : QWidget(parent)
{
    QFont titleFont;
    titleFont.setPointSize(15);
    titleFont.setBold(true);
    titleFont.setWeight(75);

    QVBoxLayout* v1 = new QVBoxLayout();

    // Create first row with position (latitude and longitude)
    {
        QLabel *latTitle = new QLabel("Lat (N/S)", this);
        QLabel *lonTitle = new QLabel("Lon (E/W)", this);
        latTitle->setFont(titleFont);
        lonTitle->setFont(titleFont);
        latTitle->setFixedWidth(90);
        lonTitle->setFixedWidth(100);

        pLatitude = new QLabel(this);
        pLongitude = new QLabel(this);

        QHBoxLayout *h1 = new QHBoxLayout();
        h1->addWidget(latTitle);
        h1->addWidget(pLatitude);
        h1->addWidget(lonTitle);
        h1->addWidget(pLongitude);

        v1->addItem(h1);
    }

    // Create second row with orientation (pitch roll yaw)
    {
        QLabel *pitchTitle = new QLabel("Pitch", this);
        QLabel *rollTitle = new QLabel("Roll", this);
        QLabel *yawTitle = new QLabel("Yaw", this);
        pitchTitle->setFont(titleFont);
        rollTitle->setFont(titleFont);
        yawTitle->setFont(titleFont);
        pitchTitle->setFixedWidth(60);
        rollTitle->setFixedWidth(60);
        yawTitle->setFixedWidth(60);

        pPitch = new QLabel(this);
        pRoll = new QLabel(this);
        pYaw = new QLabel(this);

        QHBoxLayout *h2 = new QHBoxLayout();
        h2->addWidget(pitchTitle);
        h2->addWidget(pPitch);
        h2->addWidget(rollTitle);
        h2->addWidget(pRoll);
        h2->addWidget(yawTitle);
        h2->addWidget(pYaw);

        v1->addItem(h2);
    }

    this->setMaximumHeight(70);
    this->setLayout(v1);
}

void PoseDisplay::setPitch(float value) {
    pPitch->setText(displayAngle(value));
}

void PoseDisplay::setRoll(float value) {
    pRoll->setText(displayAngle(value));
}

void PoseDisplay::setYaw(float value) {
    pYaw->setText(displayAngle(value));
}

void PoseDisplay::setLongitude(float value) {
    // TODO: Format this string beautifully
    QString str;
    pLatitude->setText(str.setNum(value, 'f', 10));
}

void PoseDisplay::setLatitude(float value) {
    // TODO: Format this string beautifully
    QString str;
    pLatitude->setText(str.setNum(value, 'f', 10));

}
