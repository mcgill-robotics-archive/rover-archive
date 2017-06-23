//
// Created by David Lavoie-Boutin on 23/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <QtWidgets/QHBoxLayout>
#include "ArmClosedLoopView.h"

ArmClosedLoopView::ArmClosedLoopView(QWidget *parent) : QWidget(parent) {
    QHBoxLayout* layout = new QHBoxLayout;
    position = new QRadioButton("Position Control", this);
    orientation = new QRadioButton("Orientation Control", this);

    layout->addWidget(position);
    layout->addWidget(orientation);
    setLayout(layout);

    connect(position, &QRadioButton::clicked, this, &ArmClosedLoopView::modeButtonClicked);
    connect(orientation, &QRadioButton::clicked, this, &ArmClosedLoopView::modeButtonClicked);
}

void ArmClosedLoopView::changeCloseLoopMode(ArmClosedLoopMode mode) {
    if (mode == POSITION)
        position->setChecked(true);
    else if (mode == ORIENTATION)
        orientation->setChecked(true);
}

void ArmClosedLoopView::modeButtonClicked() {
    if (position->isChecked())
        emit closeLoopModeChanged(POSITION);
    if (orientation->isChecked())
        emit closeLoopModeChanged(ORIENTATION);
}
