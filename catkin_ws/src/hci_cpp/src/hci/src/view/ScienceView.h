//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_SCIENCEVIEW_H
#define HCI_CPP_SCIENCEVIEW_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>
#include <ros/node_handle.h>


class ScienceView : public QWidget {
    Q_OBJECT
public:
    ScienceView(QWidget *parent);

    virtual ~ScienceView() {};

private:
    QLabel* windSpeed;
    QLabel* humidity;
    QLabel* temperature;
    QLabel* carriageSpeed;
    QLabel* carriagePosition;
    QLabel* drillSpeed;
    QLabel* drillPosition;
    QLabel* probeSpeed;
};


#endif //HCI_CPP_SCIENCEVIEW_H
