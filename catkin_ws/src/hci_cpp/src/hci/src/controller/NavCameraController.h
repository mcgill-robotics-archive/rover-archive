//
// Created by David Lavoie-Boutin on 05/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_NAVCAMERACONTROLLER_H
#define HCI_CPP_NAVCAMERACONTROLLER_H

#include <QtWidgets/QWidget>
#include <view/NavCameraView.h>
#include <rimstreamer/NyanVideoFeed.h>


class NavCameraController : public QObject {
Q_OBJECT
public:
    NavCameraController(NavCameraView* navCameraView);

    virtual ~NavCameraController() {};

private:
    NavCameraView* navCamViewPtr;

    rimstreamer::GstVideoFeedPtr topFeedPtr;
    rimstreamer::GstVideoFeedPtr leftFeedPtr;
    rimstreamer::GstVideoFeedPtr rightFeedPrt;


};


#endif //HCI_CPP_NAVCAMERACONTROLLER_H
