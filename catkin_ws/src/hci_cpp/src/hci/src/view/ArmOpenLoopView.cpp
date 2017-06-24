//
// Created by David Lavoie-Boutin on 23/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <QtWidgets/QGridLayout>
#include "ArmOpenLoopView.h"
#include <QDebug>

ArmOpenLoopView::ArmOpenLoopView(QWidget *parent) : QWidget(parent) {
    QGridLayout* grid = new QGridLayout;
    base = new QRadioButton("Base", this);
    d1 = new QRadioButton("Diff 1", this);
    d2 = new QRadioButton("Diff 2", this);
    ee = new QRadioButton("End Effector", this);

    grid->addWidget(base, 0, 0);
    grid->addWidget(d1, 0, 1);
    grid->addWidget(d2, 1, 0);
    grid->addWidget(ee, 1, 1);

    setLayout(grid);
    connect(base, &QRadioButton::clicked, this, &ArmOpenLoopView::jointButtonClicked);
    connect(d1, &QRadioButton::clicked, this, &ArmOpenLoopView::jointButtonClicked);
    connect(d2, &QRadioButton::clicked, this, &ArmOpenLoopView::jointButtonClicked);
    connect(ee, &QRadioButton::clicked, this, &ArmOpenLoopView::jointButtonClicked);
}

void ArmOpenLoopView::jointButtonClicked() {
    if (base->isChecked())
        emit armJointChanged(BASE);
    else if (d1->isChecked())
        emit armJointChanged(D1);
    else if (d2->isChecked())
        emit armJointChanged(D2);
    else if (ee->isChecked())
        emit armJointChanged(END_EFFECTOR);
}

void ArmOpenLoopView::changeArmJoint(ArmJoint joint) {
    if (joint == BASE)
        base->setChecked(true);
    else if (joint == D1)
        d1->setChecked(true);
    else if (joint == D2)
        d2->setChecked(true);
    else if (joint == END_EFFECTOR)
        ee->setChecked(true);
}
