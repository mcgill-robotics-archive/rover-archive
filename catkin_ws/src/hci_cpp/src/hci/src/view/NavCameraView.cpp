//
// Created by David Lavoie-Boutin on 05/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <QtWidgets/QHBoxLayout>
#include "NavCameraView.h"
#include <QDebug>

NavCameraView::NavCameraView(QWidget *parent) : QWidget(parent) {
    //create the 3 camera feeds, topView, leftView and rightView
    topLeftView = new SingleCameraView(this, false);
    topRightView = new SingleCameraView(this, false);
    bottomLeftView = new SingleCameraView(this, false);
    bottomRightView = new SingleCameraView(this, false);

    //create box hB of bottom layout that includes right and left cameras
    QHBoxLayout *hB = new QHBoxLayout;
    hB->setContentsMargins(0, 0, 0, 0);
    hB->addWidget(bottomLeftView);
    hB->addWidget(bottomRightView);
    
    //create box h of top layout that includes right and left cameras
    QHBoxLayout *hT = new QHBoxLayout;
    hT->setContentsMargins(0, 0, 0, 0);
    hT->addWidget(topLeftView);
    hBT->addWidget(topRightView);
 
    //create final layout, with h on top and topView camera on bottom
    QVBoxLayout *v = new QVBoxLayout;
    v->setContentsMargins(0, 0, 0, 0);
    v->addItem(hT);
    v->addItem(hB);
    //v->addWidget(topView);

    //topView->setFixedSize(1024,768);
    topLeftView->setFixedSize(480, 640);
    topRightView->setFixedSize(480, 640);
    bottomLeftView->setFixedSize(480, 640);
    bottomRightView->setFixedSize(480, 640);

    setLayout(v);
}

//
SingleCameraView *NavCameraView::screenPtr(NavCameraView::ScreenID id)
{
    switch (id)
    {
        case TOP_RIGHT:
            return topRightView;
        case TOP_LEFT:
            return topLeftView;
        case BOTTOM_LEFT:
            return bottomLeftView;
        case BOTTOM_RIGHT:
            return bottomRightView;
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

    //topView->setFixedSize(width(), 0.36 * widgetSize.height() - 5);
    topLeftView->setFixedSize(width()/2, (1-0.36) * widgetSize.height() - 2);
    topRightView->setFixedSize(width()/2, (1-0.36) * widgetSize.height() - 2);
    bottomLeftView->setFixedSize(width()/2, (1-0.36) * widgetSize.height() - 2);
    bottomRightView->setFixedSize(width()/2, (1-0.36) * widgetSize.height() - 2);
    qDebug() << topView->size();
}
