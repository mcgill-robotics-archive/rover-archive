//
// Created by David Lavoie-Boutin on 30/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "VideoView.h"

using rimstreamer::VideoFeedPtr;
using rimstreamer::GstVideoFeedPtr;

VideoView::VideoView(QWidget *parent) : QWidget(parent) {
    topScreen = new rimstreamer::VideoFeedWidget(this);
    topScreen->setFixedSize(320, 180);
}

void VideoView::changeTopFeed(const GstVideoFeedPtr &videoFeed) {

    VideoFeedPtr feed = topScreen->getVideoFeed();
    if (feed && feed == videoFeed)
        return;

    //If there is a feed running, stop it.
    if (feed)
    {
        feed->stop();
        topScreen->releaseVideoFeed();
    }

    qDebug() << "Setting feed" << videoFeed->toString();
    //Play the feed
    topScreen->setVideoFeed(videoFeed);
    videoFeed->play();
}
