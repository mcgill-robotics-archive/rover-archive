//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_CAMERAVIEW_H
#define HCI_CPP_CAMERAVIEW_H

#include <QtWidgets/QWidget>


class CameraView : public QWidget {
Q_OBJECT
public:
    CameraView(QWidget *parent = 0);

    virtual ~CameraView() {};
};


#endif //HCI_CPP_CAMERAVIEW_H
