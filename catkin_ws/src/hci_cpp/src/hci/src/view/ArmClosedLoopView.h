//
// Created by David Lavoie-Boutin on 23/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_ARMCLOSEDLOOPVIEW_H
#define HCI_CPP_ARMCLOSEDLOOPVIEW_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QRadioButton>
#include <model/ArmData.h>


class ArmClosedLoopView : public QWidget {
Q_OBJECT
public:
    ArmClosedLoopView(QWidget *parent = 0);
    virtual ~ArmClosedLoopView() {};

public slots:
    void changeCloseLoopMode(ArmClosedLoopMode mode);

signals:
    void closeLoopModeChanged(ArmClosedLoopMode mode);

private:
    QRadioButton* position;
    QRadioButton* orientation;

private slots:
    void modeButtonClicked();
};


#endif //HCI_CPP_ARMCLOSEDLOOPVIEW_H
