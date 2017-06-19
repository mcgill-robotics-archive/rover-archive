//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_SCIENCEVIEW_H
#define HCI_CPP_SCIENCEVIEW_H

#include <QtWidgets/QWidget>


class ScienceView : public QWidget {
    Q_OBJECT
public:
    ScienceView(QWidget *parent = 0);

    virtual ~ScienceView() {};
};


#endif //HCI_CPP_SCIENCEVIEW_H
