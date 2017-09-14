//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_CAMERAVIEW_H
#define HCI_CPP_CAMERAVIEW_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QComboBox>
#include <rimstreamer/VideoFeedWidget.h>

using namespace rimstreamer;

class SingleCameraView : public QWidget {
Q_OBJECT
public:
    SingleCameraView(QWidget *parent= nullptr, bool showSelector=false, VideoFeedWidget::Orientation orientation=VideoFeedWidget::NONE);

    virtual ~SingleCameraView() = default;

    void setVideoFeed(const GstVideoFeedPtr& feed);
    void releaseVideoFeed();
    VideoFeedPtr getVideoFeed();


private:
    QComboBox* mAvailableList;
    rimstreamer::VideoFeedWidget* mScreenWidget;
};


#endif //HCI_CPP_CAMERAVIEW_H
