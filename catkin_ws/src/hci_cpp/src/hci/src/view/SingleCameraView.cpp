//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//


#include "SingleCameraView.h"
#include <QtGlobal>

SingleCameraView::SingleCameraView(QWidget *parent, bool showSelector, int angle, int ind, VideoFeedWidget::Orientation orientation) : QWidget(parent) {
    mAvailableList = new QComboBox(this);
    //mScreenWidget = setAngle(angle);//new rimstreamer::VideoFeedWidget(this, 0, orientation);
    rimstreamer::VideoFeedWidget* feedWidget = setAngle(angle);
    //Added
    angleSelector = new CameraAngleSelect(this, angle);
    imageDisplay = new QLabel(this);
    showListAngle = showSelector;

    setList();
    mAvailableList->setCurrentIndex(ind);
    camAngle = angle;

    
    //connect mAvailableList with indexIsChanged (will emit indexChanged signal)
    connect(mAvailableList, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &SingleCameraView::indexIsChanged);

    //connect angelSelector with changeAngle (using signal angleChanged(int) from CameraAngelSelect.cpp)
    connect(angleSelector, &CameraAngleSelect::angleChanged, this, &SingleCameraView::changeAngle);

    sizePolicy = QSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    mAvailableList->setSizePolicy(sizePolicy);
    angleSelector->setSizePolicy(sizePolicy);
    
    layout = new QVBoxLayout;
    settings = new QHBoxLayout;
    newLayout();
    /*//layout of camer view, including the combo box mAvailableList and the mScreenWidget
    QVBoxLayout* layout = new QVBoxLayout;
    QHBoxLayout* settings = new QHBoxLayout;

    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(mScreenWidget);
    settings->addWidget(mAvailableList);
    settings->addWidget(angleSelector);
    layout->addItem(settings);
 
    //if the cameras are supposed to show the mAvailableList/angleSelector, do so, otherwise, hide it
    if (showSelector){
        mAvailableList->show();
        angleSelector->show();
    }
    else{
        mAvailableList->hide();
        angleSelector->hide();
    }

    setLayout(layout);*/
}

void SingleCameraView::setVideoFeed(const GstVideoFeedPtr &feed) {
    mScreenWidget->setVideoFeed(feed);
}

void SingleCameraView::releaseVideoFeed() {
    mScreenWidget->releaseVideoFeed();
}

VideoFeedPtr SingleCameraView::getVideoFeed() {
    return mScreenWidget->getVideoFeed();
}


void SingleCameraView::indexIsChanged(int index){
    ROS_INFO("The index was changed");
    
    emit indexChanged(index);
} 

void SingleCameraView::setList(){
    QStringList * list = new QStringList;
    /*
    list->append("Camera 1");    //index 0
    list->append("Camera 2");    //index 1
    list->append("Camera 3");
    list->append("Camera 4");
    list->append("Hikcamera 5");
    list->append("Hikcamera 6"); //index 5
    */
    list->append("Bottom");
    list->append("Top Left");
    list->append("Top Right");
    mAvailableList->addItems(*list);
}

void SingleCameraView::topicCallback(int index){
/*if index < self._topic_selector.count():
            topic = self._topic_selector.itemText(index)
            self._active_topic = topic
            self.playTopic.emit(topic)
*/
    if(index < mAvailableList->count()){
        topic = mAvailableList->itemText(index);
        activeTopic = topic;
    }
}

rimstreamer::VideoFeedWidget* SingleCameraView::setAngle(int angle){
    //rimstreamer::VideoFeedWidget* newScreenWidget;
    camAngle = angle;
    if (camAngle == 90) {
        mScreenWidget = new rimstreamer::VideoFeedWidget(this, 0, VideoFeedWidget::CW);
    }
    else if (camAngle == 270) {
        mScreenWidget = new rimstreamer::VideoFeedWidget(this, 0, VideoFeedWidget::CCW);
    }
    else {
        mScreenWidget = new rimstreamer::VideoFeedWidget(this, 0, VideoFeedWidget::NONE);
    }
    return mScreenWidget;
}

void SingleCameraView::changeAngle(int angle){
    //rimstreamer::VideoFeedWidget* thisScreenWidget;
    camAngle = angle;
    if (camAngle == 90) {
        delete mScreenWidget;
        mScreenWidget = new rimstreamer::VideoFeedWidget(this, 0, VideoFeedWidget::CW);
    }
    else if (camAngle == 270) {
        mScreenWidget = new rimstreamer::VideoFeedWidget(this, 0, VideoFeedWidget::CCW);
    }
    else {
        mScreenWidget = new rimstreamer::VideoFeedWidget(this, 0, VideoFeedWidget::NONE);
    }
    //mScreenWidget = thisScreenWidget;
    delete layout;
    delete settings;
    newLayout();
}

void SingleCameraView::newLayout(){
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(mScreenWidget);
    settings->addWidget(mAvailableList);
    settings->addWidget(angleSelector);
    layout->addItem(settings);
 
    //if the cameras are supposed to show the mAvailableList/angleSelector, do so, otherwise, hide it
    if (showListAngle){
        mAvailableList->show();
        angleSelector->show();
    }
    else{
        mAvailableList->hide();
        angleSelector->hide();
    }

    setLayout(layout);
}
    




    
