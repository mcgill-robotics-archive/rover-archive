//
// Created by David Lavoie-Boutin on 05/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_NAVCAMERAVIEW_H
#define HCI_CPP_NAVCAMERAVIEW_H

#include <QtWidgets/QWidget>
#include "SingleCameraView.h"

using namespace rimstreamer;

class NavCameraView : public QWidget {
Q_OBJECT
public:
    NavCameraView(QWidget *parent = 0);

    virtual ~NavCameraView() {};

    enum ScreenID
    {
        TOP_RIGHT = 1,
        TOP_LEFT = 2,
        BOTTOM_LEFT = 3,
        BOTTOM_RIGHT = 4
    };

    void setVideoFeed(const GstVideoFeedPtr& feed, ScreenID id);
    void releaseVideoFeed(ScreenID id);
    VideoFeedPtr getVideoFeed(ScreenID id);

protected:
    virtual void resizeEvent(QResizeEvent* event);

private:
    SingleCameraView* topRightView;
    SingleCameraView* topLeftView;
    SingleCameraView* bottomLeftView;
    SingleCameraView* bottomRightView;

    SingleCameraView* screenPtr(ScreenID id);
};


#endif //HCI_CPP_NAVCAMERAVIEW_H
