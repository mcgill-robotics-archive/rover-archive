//
// Created by David Lavoie-Boutin on 05/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <QtWidgets/QHBoxLayout>
#include "NavCameraView.h"
#include <QDebug>

NavCameraView::NavCameraView(QWidget *parent) : QWidget(parent) {
    //create the 3 camera feeds, topView, leftView and rightView

    topView = new SingleCameraView(this, true, 0, 0);
    bottomLeftView = new SingleCameraView(this, true, 270, 1);
    bottomRightView = new SingleCameraView(this, true, 9, 2);
/*
    topLeftView = new SingleCameraView(this, false, 270);
    topRightView = new SingleCameraView(this, false, 90);
    bottomLeftView = new SingleCameraView(this, false, 270);
    bottomRightView = new SingleCameraView(this, false, 90);
*/

    //create box hB of bottom layout that includes right and left cameras
    QHBoxLayout *hB = new QHBoxLayout;
    hB->setContentsMargins(0, 0, 0, 0);
    hB->addWidget(bottomLeftView);
    hB->addWidget(bottomRightView);
    
/*
    //create box h of top layout that includes right and left cameras
    QHBoxLayout *hT = new QHBoxLayout;
    hT->addWidget(topLeftView);
    hT->addWidget(topRightView);
*/
 
    //create final layout, with h on top and topView camera on bottom
    v = new QVBoxLayout;
    v->setContentsMargins(0, 0, 0, 0);
    //v->addItem(hT);
    v->addItem(hB);
    v->addWidget(topView);

    topView->setFixedSize(1024,768);
    //topLeftView->setFixedSize(480, 640);
    //topRightView->setFixedSize(480, 640);
    bottomLeftView->setFixedSize(480, 640);
    bottomRightView->setFixedSize(480, 640);

    setLayout(v);

    //connect cameras to control feeds
    connect(topView, &SingleCameraView::indexChanged, this, &NavCameraView::topChangedIndex);
    connect(bottomLeftView, &SingleCameraView::indexChanged, this, &NavCameraView::bottomLeftChangedIndex);
    connect(bottomRightView, &SingleCameraView::indexChanged, this, &NavCameraView::bottomRightChangedIndex);
}


SingleCameraView *NavCameraView::screenPtr(NavCameraView::ScreenID id)
{
    switch (id)
    {
        case TOP:
            return topView;
        //case TOP_RIGHT:
            //return topRightView;
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

    topView->setFixedSize(width(), 0.36 * widgetSize.height() - 5);
    //topLeftView->setFixedSize(width()/2, (1-0.36) * widgetSize.height() - 2);
    //topRightView->setFixedSize(width()/2, (1-0.36) * widgetSize.height() - 2);
    bottomLeftView->setFixedSize(width()/2, (1-0.36) * widgetSize.height() - 2);
    bottomRightView->setFixedSize(width()/2, (1-0.36) * widgetSize.height() - 2);
    qDebug() << topView->size();
}

int NavCameraView::changedIndex(int index){
    return index;
}


void NavCameraView::topChangedIndex(int index){
    int ind = index;
    changeFeed(topView, ind); 
}

void NavCameraView::bottomLeftChangedIndex(int index){
    int ind = index;
    changeFeed(bottomLeftView, ind); 
}

void NavCameraView::bottomRightChangedIndex(int index){
    int ind = index;
    changeFeed(bottomRightView, ind); 
}

/*input: QWidget baseFeed, int index
  output: none, effect is to change the layout
  Comments: translate the index to the opsition you want the baseFeed to be in (called newFeed)
               swap the feeds and update the layout 
*/

void NavCameraView::changeFeed(QWidget* baseFeed, int index){
    QWidget *newFeed;
    if (index == 0)
         newFeed = topView;//new SingleCameraView(this, true, 0, 0);
    else if (index == 1)
         newFeed = bottomLeftView;//new SingleCameraView(this, true, 0, 1);
    else if (index == 2)
         newFeed = bottomRightView;//new SingleCameraView(this, true, 0, 2);
    v->replaceWidget(baseFeed, newFeed);
    //v->replaceWidget(newFeed, baseFeed);
    v->update();    
}












