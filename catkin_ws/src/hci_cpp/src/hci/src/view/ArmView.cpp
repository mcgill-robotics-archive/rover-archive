//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QButtonGroup>
#include "ArmView.h"
#include <QDebug>

ArmView::ArmView(QWidget *parent) : QWidget(parent) {
    QHBoxLayout* h1 = new QHBoxLayout;
    QVBoxLayout* v1 = new QVBoxLayout;
    QButtonGroup* buttonGroup = new QButtonGroup;

    openMode = new QPushButton("Open Loop Mode");
    closedMode = new QPushButton("Closed Loop Mode");
    openMode->setCheckable(true);
    closedMode->setCheckable(true);
    h1->addWidget(openMode);
    h1->addWidget(closedMode);
    buttonGroup->addButton(openMode);
    buttonGroup->addButton(closedMode);
    buttonGroup->setExclusive(true);

    openView = new ArmOpenLoopView(this);
    closedView = new ArmClosedLoopView(this);

    connect(openView, &ArmOpenLoopView::armJointChanged, this, &ArmView::armJointChanged);
    connect(closedView, &ArmClosedLoopView::closeLoopModeChanged, this, &ArmView::closeLoopModeChanged);
    connect(openMode, &QPushButton::clicked, this, &ArmView::buttonModeClicked);
    connect(closedMode, &QPushButton::clicked, this, &ArmView::buttonModeClicked);

    v1->addItem(h1);
    v1->addWidget(closedView);
    v1->addWidget(openView);
    setLayout(v1);
    closedView->setDisabled(true);
    openView->setDisabled(true);

}

void ArmView::setArmMode(ArmMode mode) {
    if (mode == OPEN)
    {
        closedView->hide();
        openView->show();
        closedView->setDisabled(true);
        openView->setDisabled(false);
        openMode->setChecked(true);
        closedMode->setChecked(false);
    }
    else if (mode == CLOSED)
    {
        closedView->show();
        openView->hide();
        closedView->setDisabled(false);
        openView->setDisabled(true);
        openMode->setChecked(false);
        closedMode->setChecked(true);
    }
}

void ArmView::changeArmJoint(ArmJoint joint) {
    openView->changeArmJoint(joint);
}

void ArmView::changeCloseLoopMode(ArmClosedLoopMode mode) {
    closedView->changeCloseLoopMode(mode);
}

void ArmView::buttonModeClicked() {
    if (openMode->isChecked())
    {
        setArmMode(OPEN);
        emit armModeChanged(OPEN);
    }
    else if (closedMode->isChecked())
    {
        setArmMode(CLOSED);
        emit armModeChanged(CLOSED);
    }
}
