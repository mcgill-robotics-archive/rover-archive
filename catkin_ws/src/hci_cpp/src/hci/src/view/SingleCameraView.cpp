//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <QtWidgets/QVBoxLayout>
#include "SingleCameraView.h"

SingleCameraView::SingleCameraView(QWidget *parent, bool showSelector, VideoFeedWidget::Orientation orientation) : QWidget(parent) {
    mAvailableList = new QComboBox(this);
    mScreenWidget = new rimstreamer::VideoFeedWidget(this, 0, orientation);

    //Added
    int curr_topic;

    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(mScreenWidget);
    layout->addWidget(mAvailableList);
    //set_mAvailable();

    if (showSelector)

        mAvailableList->show();
    else
        mAvailableList->hide();
    
    mAvailableList->activated(mAvailableList->currentIndex());

    //mAvailableList->activated(set_Feed);


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

void SingleCameraView::set_mAvailable(QStringList lst) {
   mAvailableList->clear();
   //QStringList lst;
   //lst.insert(0,"one");
   //lst.insert(2,"two");
   mAvailableList->addItems (lst);
}

int SingleCameraView::set_Feed(int index) {
    if (index < mAvailableList->count()) {
        int topic =index; 
        curr_topic = topic;
    }
    return curr_topic;
}

    
