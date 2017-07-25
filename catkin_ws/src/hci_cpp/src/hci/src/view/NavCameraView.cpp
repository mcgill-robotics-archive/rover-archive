//
// Created by David Lavoie-Boutin on 05/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <QtWidgets/QHBoxLayout>
#include "NavCameraView.h"
#include <QDebug>

NavCameraView::NavCameraView(QWidget *parent) : QWidget(parent) {
    topView = new SingleCameraView(this, false);
    leftView = new SingleCameraView(this, false, VideoFeedWidget::CW);
    rightView = new SingleCameraView(this, false, VideoFeedWidget::CCW);

    QHBoxLayout *h = new QHBoxLayout;
    h->setContentsMargins(0, 0, 0, 0);
    h->addWidget(leftView);
    h->addWidget(rightView);

    QVBoxLayout *v = new QVBoxLayout;
    v->setContentsMargins(0, 0, 0, 0);
    v->addItem(h);
    v->addWidget(topView);

    setLayout(v);
}

SingleCameraView *NavCameraView::screenPtr(NavCameraView::ScreenID id)
{
    switch (id)
    {
        case TOP:
            return topView;
        case LEFT:
            return leftView;
        case RIGHT:
            return rightView;
        default:
            return nullptr;
    }
}

void NavCameraView::setVideoFeed(const GstVideoFeedPtr &feed, ScreenID id) {
    SingleCameraView * screenPtr1 = screenPtr(id);
    screenPtr1->setVideoFeed(feed);
}

void NavCameraView::releaseVideoFeed(ScreenID id) {
    SingleCameraView * screenPtr1 = screenPtr(id);
    screenPtr1->releaseVideoFeed();
}

VideoFeedPtr NavCameraView::getVideoFeed(ScreenID id) {
    SingleCameraView * screenPtr1 = screenPtr(id);
    return screenPtr1->getVideoFeed();
}

void NavCameraView::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);

    QSize widgetSize(size());
    qDebug() << widgetSize;

    topView->setFixedSize(width(), 0.36 * widgetSize.height() - 5);
    leftView->setFixedSize(width()/2, (1-0.36) * widgetSize.height() - 2);
    rightView->setFixedSize(width()/2, (1-0.36) * widgetSize.height() - 2);
    qDebug() << topView->size();
}
