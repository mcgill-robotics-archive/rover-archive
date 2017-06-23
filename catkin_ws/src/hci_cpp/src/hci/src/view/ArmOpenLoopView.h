//
// Created by David Lavoie-Boutin on 23/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_ARMOPENLOOPVIEW_H
#define HCI_CPP_ARMOPENLOOPVIEW_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QRadioButton>
#include <model/ArmData.h>


class ArmOpenLoopView : public QWidget {
Q_OBJECT
public:
    ArmOpenLoopView(QWidget *parent = 0);
    virtual ~ArmOpenLoopView() {};

public slots:
    void changeArmJoint(ArmJoint joint);

signals:
    void armJointChanged(ArmJoint joint);

private:
    QRadioButton* base;
    QRadioButton* d1;
    QRadioButton* d2;
    QRadioButton* ee;

private slots:
    void jointButtonClicked();
};


#endif //HCI_CPP_ARMOPENLOOPVIEW_H
