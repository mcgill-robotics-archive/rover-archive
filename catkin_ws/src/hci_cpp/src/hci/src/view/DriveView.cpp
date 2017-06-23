//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QButtonGroup>
#include "DriveView.h"
#include "utilities.h"

DriveView::DriveView(QWidget *parent) : QWidget(parent) {
    QVBoxLayout* vbox1 = new QVBoxLayout;
    vbox1->setContentsMargins(0, 0, 0, 0);

    // Create drive mode selection
    {
        QLabel* modeTitle = new QLabel("Active Steering mode: ", this);
        pActiveSetteringMode = new QLabel("None", this);
        QHBoxLayout* hbox1 = new QHBoxLayout;
        hbox1->setContentsMargins(0, 0, 0, 0);
        hbox1->addWidget(modeTitle);
        hbox1->addWidget(pActiveSetteringMode);

        pEnableAckermann = new QPushButton("Ackermann", this);
        pEnableAckermann->setCheckable(true);
        pEnablePoint = new QPushButton("Point Steering", this);
        pEnablePoint->setCheckable(true);
        pEnableTranslate = new QPushButton("Translation", this);
        pEnableTranslate->setCheckable(true);

        QHBoxLayout* hbox2 = new QHBoxLayout;
        hbox2->setContentsMargins(0, 0, 0, 0);
        hbox2->addWidget(pEnableAckermann);
        hbox2->addWidget(pEnablePoint);
        hbox2->addWidget(pEnableTranslate);

        QButtonGroup* modeButtonGroup = new QButtonGroup(this);
        modeButtonGroup->addButton(pEnableAckermann);
        modeButtonGroup->addButton(pEnablePoint);
        modeButtonGroup->addButton(pEnableTranslate);
        modeButtonGroup->setExclusive(true);

        QVBoxLayout* vbox2 = new QVBoxLayout;
        vbox2->setContentsMargins(0, 0, 0, 0);
        vbox2->addItem(hbox1);
        vbox2->addItem(hbox2);
        vbox1->addItem(vbox2);

        connect(modeButtonGroup, static_cast<void(QButtonGroup::*)(QAbstractButton *)>(&QButtonGroup::buttonClicked),
                this, &DriveView::onModeButtonClick);
    }

    QFrame* line_1 = new QFrame(this);
    line_1->setFrameShape(QFrame::HLine);
    line_1->setFrameShadow(QFrame::Sunken);
    vbox1->addWidget(line_1);

    // Create motor status indicator grid
    {
        QLabel *statusTitle = new QLabel("Motor Connection Status:", this);
        vbox1->addWidget(statusTitle);

        pFLStatus = new QLabel("Status", this);
        pFLStatus->setAlignment(Qt::AlignCenter);
        pFRStatus = new QLabel("Status", this);
        pFRStatus->setAlignment(Qt::AlignCenter);
        pMLStatus = new QLabel("Status", this);
        pMLStatus->setAlignment(Qt::AlignCenter);
        pMRStatus = new QLabel("Status", this);
        pMRStatus->setAlignment(Qt::AlignCenter);
        pBLStatus = new QLabel("Status", this);
        pBLStatus->setAlignment(Qt::AlignCenter);
        pBRStatus = new QLabel("Status", this);
        pBRStatus->setAlignment(Qt::AlignCenter);

        QGridLayout *statusGrid = new QGridLayout;
        statusGrid->setContentsMargins(0, 0, 0, 0);
        statusGrid->addWidget(pFLStatus, 0, 0);
        statusGrid->addWidget(pMLStatus, 1, 0);
        statusGrid->addWidget(pBLStatus, 2, 0);
        statusGrid->addWidget(pFRStatus, 0, 1);
        statusGrid->addWidget(pMRStatus, 1, 1);
        statusGrid->addWidget(pBRStatus, 2, 1);
        vbox1->addItem(statusGrid);
    }

    this->setLayout(vbox1);
}

void DriveView::updateDriveStatus(const DriveStatusData &statusData) {
    statusData.flGood ? setOkBG(pFLStatus) : setBadBG(pFLStatus);
    statusData.frGood ? setOkBG(pFRStatus) : setBadBG(pFRStatus);
    statusData.mlGood ? setOkBG(pMLStatus) : setBadBG(pMLStatus);
    statusData.mrGood ? setOkBG(pMRStatus) : setBadBG(pMRStatus);
    statusData.blGood ? setOkBG(pBLStatus) : setBadBG(pBLStatus);
    statusData.brGood ? setOkBG(pBRStatus) : setBadBG(pBRStatus);
}

void DriveView::onModeButtonClick(const QAbstractButton *button) {
    SteeringMode mode;
    if (button == pEnableAckermann)
        mode = ACKERMANN;
    else if (button == pEnablePoint)
        mode = POINT;
    else
        mode = TRANSLATE;

    emit steeringModeChanged(mode);
}

void DriveView::updateSteeringMode(const SteeringMode &mode) {
    if (mode == ACKERMANN)
    {
        pActiveSetteringMode->setText("ACKERMANN");
        pEnableAckermann->setChecked(true);
    }
    else if (mode == POINT)
    {
        pActiveSetteringMode->setText("POINT STEERING");
        pEnablePoint->setChecked(true);
    }
    else
    {
        pActiveSetteringMode->setText("TRANSLATION");
        pEnableTranslate->setChecked(true);
    }
}
