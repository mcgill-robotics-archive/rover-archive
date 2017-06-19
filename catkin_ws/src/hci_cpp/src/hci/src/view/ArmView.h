//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_ARMVIEW_H
#define HCI_CPP_ARMVIEW_H

#include <QtWidgets/QWidget>


class ArmView : public QWidget {
Q_OBJECT
public:
    ArmView(QWidget *parent = 0);

    virtual ~ArmView() {};
};


#endif //HCI_CPP_ARMVIEW_H
