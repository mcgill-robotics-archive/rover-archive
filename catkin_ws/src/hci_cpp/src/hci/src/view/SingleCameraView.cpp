//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//


#include "SingleCameraView.h"

SingleCameraView::SingleCameraView(QWidget *parent, bool showSelector, int angle, VideoFeedWidget::Orientation orientation) : QWidget(parent) {
    mAvailableList = new QComboBox(this);
    mScreenWidget = new rimstreamer::VideoFeedWidget(this, 0, orientation);
    //Added
    angleSelector = new CameraAngleSelect(this, angle);
    imageDisplay = new QLabel(this);


    sizePolicy = QSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    mAvailableList->setSizePolicy(sizePolicy);
    angleSelector->setSizePolicy(sizePolicy);
    

    //layout of camer view, including the combo box mAvailableList and the mScreenWidget
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
    

//to Add (translate from Python)
/*
    self._angle_selector.turnAngle.connect(self.set_angle)
    self._topic_selector.activated.connect(self._topic_sel_callback)
*/

    setLayout(layout);
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

//added
void SingleCameraView::set_angle(double ang){
    angle = ang;
    angleSelector->defaultAngle(ang);
}

void SingleCameraView::newSample(QImage image){
    if (image.isNull())
        imageDisplay->setText("No image");

    else{
        if (angle != 0)
            imageNew = image.transformed(QTransform().rotate(angle), Qt::SmoothTransformation);
        else
            imageNew = image;
        imageNew = imageNew.scaled(this->width(), this->height(), Qt::KeepAspectRatio, Qt::FastTransformation); 
        pixmap = QPixmap::fromImage(imageNew);
        imageDisplay->setPixmap(pixmap);
    } 
}

void SingleCameraView::addEntry(QString string){
    if( !string.isNull())
        mAvailableList->addItem(string);
}

void SingleCameraView::setList(QStringList list){ 
    mAvailableList->clear();
    mAvailableList->addItems(list);
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

/*void SingleCameraView::set_mAvailable(QStringList lst) {
   mAvailableList->clear();

   mAvailableList->addItems (lst);
}

int SingleCameraView::set_Feed(int index) {
    if (index < mAvailableList->count()) {
        int topic =index; 
        curr_topic = topic;
    }
    return curr_topic;
}*/
        



    
