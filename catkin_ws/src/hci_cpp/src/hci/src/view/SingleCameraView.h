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
#include "CameraAngleSelect.h"
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>
#include <QSizePolicy>
#include <QImage>
#include <QLabel>
#include <QTransform> 
#include <QPixmap> 
#include <QString>

using namespace rimstreamer;

class SingleCameraView : public QWidget {
Q_OBJECT


public:
    SingleCameraView(QWidget *parent= nullptr, bool showSelector=false, int angle=0, int ind=0, VideoFeedWidget::Orientation orientation=VideoFeedWidget::NONE);

    virtual ~SingleCameraView() = default;

    void setVideoFeed(const GstVideoFeedPtr& feed);
    void releaseVideoFeed();
    VideoFeedPtr getVideoFeed();

//added
    void set_mAvailable(QStringList lst);
    int set_Feed(int index);
    //void set_angle(double ang);
    void newSample(QImage image); 
    void addEntry(QString string);
    void setList();//QStringList list);
    void topicCallback(int index);
    void indexIsChanged(int index);
    rimstreamer::VideoFeedWidget* setAngle(int angle);
    void changeAngle(int angle);
    void newLayout();

signals:
    void indexChanged(int index);

private:
    QComboBox* mAvailableList;
    rimstreamer::VideoFeedWidget* mScreenWidget;
    QLabel* imageDisplay;
    CameraAngleSelect *angleSelector;
    QString activeTopic;
//added
    QSizePolicy sizePolicy;
    int curr_topic; 
    int camAngle;
    bool showListAngle;
    QImage imageNew;
    QPixmap pixmap;
    QString topic;
    QVBoxLayout* layout;
    QHBoxLayout* settings;


};


#endif //HCI_CPP_CAMERAVIEW_H
