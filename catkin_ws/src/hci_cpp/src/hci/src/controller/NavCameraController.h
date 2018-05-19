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
    void changeFeed(NavCameraView::ScreenID id, int index);
    void stopFeedOf(NavCameraView::ScreenID id, int ind);

private:
    NavCameraView* navCamViewPtr;

    rimstreamer::GstVideoFeedPtr feed1Ptr;
    rimstreamer::GstVideoFeedPtr feed2Ptr;
    rimstreamer::GstVideoFeedPtr feed3Ptr;

    int topInd; //keep track of the current feed at TOP, bottomRight, etc. 
    int bottomRightInd;
    int bottomLeftInd;
};


#endif //HCI_CPP_NAVCAMERACONTROLLER_H
