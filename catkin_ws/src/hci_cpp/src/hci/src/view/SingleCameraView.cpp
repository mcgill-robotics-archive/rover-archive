//
// Created by David Lavoie-Boutin on 18/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <QtWidgets/QVBoxLayout>
#include "SingleCameraView.h"

SingleCameraView::SingleCameraView(QWidget *parent, bool showSelector, VideoFeedWidget::Orientation orientation) : QWidget(parent) {
    mAvailableList = new QComboBox(this);
    mScreenWidget = new rimstreamer::VideoFeedWidget(this, 0, orientation);

    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(mScreenWidget);
    layout->addWidget(mAvailableList);

    if (showSelector)
        mAvailableList->show();
    else
        mAvailableList->hide();

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
