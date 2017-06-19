//
// Created by david on 18/06/17.
//

#include <QtWidgets/QVBoxLayout>
#include "MainView.h"
#include "AttitudeView.h"
#include "PowerSupplyInformation.h"

MainView::MainView(QWidget *parent) : QWidget(parent) {
    QVBoxLayout * vbox = new QVBoxLayout;

    QFrame* line_1 = new QFrame(this);
    line_1->setFrameShape(QFrame::HLine);
    line_1->setFrameShadow(QFrame::Sunken);

    AttitudeView* attitudeView = new AttitudeView(this);
    PowerSupplyInformation* powerSupplyInformation = new PowerSupplyInformation(this);

    vbox->addWidget(attitudeView);
    vbox->addWidget(line_1);
    vbox->addWidget(powerSupplyInformation);

    setLayout(vbox);
}
