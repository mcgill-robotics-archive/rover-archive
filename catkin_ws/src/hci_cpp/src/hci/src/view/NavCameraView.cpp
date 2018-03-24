//
// Created by David Lavoie-Boutin on 05/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <QtWidgets/QHBoxLayout>
#include "NavCameraView.h"
#include <QDebug>

NavCameraView::NavCameraView(QWidget *parent) : QWidget(parent) {
    //create the 3 camera feeds, topView, leftView and rightView
    topView = new SingleCameraView(this, true, 0);
    leftView = new SingleCameraView(this, true, 270);
    rightView = new SingleCameraView(this, true, 90);

    //create box h of layout that includes right and left cameras
    QHBoxLayout *h = new QHBoxLayout;
    h->setContentsMargins(0, 0, 0, 0);
    h->addWidget(leftView);
    h->addWidget(rightView);
 
    //create final layout, with h on top and topView camera on bottom
    QVBoxLayout *v = new QVBoxLayout;
    v->setContentsMargins(0, 0, 0, 0);
    v->addItem(h);
    v->addWidget(topView);

    topView->setFixedSize(1024,768);
    leftView->setFixedSize(480, 640);
    rightView->setFixedSize(480, 640);

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
