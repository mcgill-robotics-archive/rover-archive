//
// Created by dd8641 on 2016-07-06.
//

#ifndef RIMSTREAMER_CUSTOMVIDEOSURFACE_H
#define RIMSTREAMER_CUSTOMVIDEOSURFACE_H

#include <QObject>
#include <QGraphicsView>
#include <QGst/global.h>
#include <QSet>
#include <QGst/ElementFactory>
#include "CustomGraphicsWidget.h"
#include "CustomSink.h"

namespace rimstreamer
{
class CustomVideoSurface : public QObject
{
public:
    explicit CustomVideoSurface(QGraphicsView *parent);
    virtual ~CustomVideoSurface();

    /*! Returns the video sink element that provides this surface's image.
     * The element will be constructed the first time that this function
     * is called. The surface will always keep a reference to this element.
     */
    QGst::ElementPtr videoSink();
    QSet<CustomGraphicsWidget*> items;
    CustomSink *customSink();

//private:

    void onUpdate();
    QGraphicsView *view;
    QGst::ElementPtr mVideoSink;
    CustomSink *mCustomSink;

};
}

#endif //RIMSTREAMER_CUSTOMVIDEOSURFACE_H
