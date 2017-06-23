//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_ARMVIEW_H
#define HCI_CPP_ARMVIEW_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QPushButton>
#include "ArmClosedLoopView.h"
#include "ArmOpenLoopView.h"


class ArmView : public QWidget {
Q_OBJECT
public:
    ArmView(QWidget *parent = 0);
    virtual ~ArmView() {};

public slots:
    void setArmMode(ArmMode mode);
    void changeArmJoint(ArmJoint joint);
    void changeCloseLoopMode(ArmClosedLoopMode mode);

signals:
    void closeLoopModeChanged(ArmClosedLoopMode mode);
    void armJointChanged(ArmJoint joint);
    void armModeChanged(ArmMode mode);

private:
    QPushButton* openMode;
    QPushButton* closedMode;
    ArmClosedLoopView* closedView;
    ArmOpenLoopView* openView;

private slots:
    void buttonModeClicked();
};


#endif //HCI_CPP_ARMVIEW_H
