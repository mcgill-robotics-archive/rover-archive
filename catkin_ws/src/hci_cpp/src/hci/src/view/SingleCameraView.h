//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//
//called in: NavCamera.h

#ifndef HCI_CPP_CAMERAVIEW_H
#define HCI_CPP_CAMERAVIEW_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QComboBox>
#include <rimstreamer/VideoFeedWidget.h>
//added 
#include <QSizePolicy>

using namespace rimstreamer;

class SingleCameraView : public QWidget {
Q_OBJECT
public:
    SingleCameraView(QWidget *parent= nullptr, bool showSelector=false, VideoFeedWidget::Orientation orientation=VideoFeedWidget::NONE);

    virtual ~SingleCameraView() = default;

    void setVideoFeed(const GstVideoFeedPtr& feed);
    void releaseVideoFeed();
    VideoFeedPtr getVideoFeed();

//added
   // void set_mAvailable(QStringList lst);
    //int set_Feed(int index);


private:
    QComboBox* mAvailableList;
    rimstreamer::VideoFeedWidget* mScreenWidget;

//added
//    int curr_topic;
    
};


#endif //HCI_CPP_CAMERAVIEW_H
