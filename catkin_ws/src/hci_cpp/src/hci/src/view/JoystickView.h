//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_JOYSTICKVIEW_H
#define HCI_CPP_JOYSTICKVIEW_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QButtonGroup>


class JoystickView : public QWidget {
Q_OBJECT
public:
    JoystickView(QWidget *parent = 0);

    virtual ~JoystickView() {};

public slots:
    void addMode(QString name);

private:
    unsigned short mJoystickControllerCount;
    QGridLayout* pLayout;
    QButtonGroup* pButtonGroup;

signals:
    void modeChanged(QString modeText);

};


#endif //HCI_CPP_JOYSTICKVIEW_H
