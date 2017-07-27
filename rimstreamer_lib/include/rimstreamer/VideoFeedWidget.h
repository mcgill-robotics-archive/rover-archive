/*
 * VideoFeedWidget.h
 *
 *  Created on: 2016-03-28
 *      Author: rml00
 */

#ifndef RIMSTREAMER_VIDEOFEEDWIDGET_H_
#define RIMSTREAMER_VIDEOFEEDWIDGET_H_

#include "rimstreamer/GstVideoFeed.h"

#include <QGst/Ui/VideoWidget>

namespace rimstreamer
{

class VideoFeedWidget: public QGst::Ui::VideoWidget
{
Q_OBJECT

public:

    enum Orientation
    {
        NONE=0,
        CCW=1,
        CW=2
    };

    VideoFeedWidget(QWidget *parent = 0, Qt::WindowFlags f = 0, Orientation orientation=NONE);
    virtual ~VideoFeedWidget();

    /**
     * Returns the VideoFeed registered to the widget.
     * May be NULL if none was registered.
     */
    VideoFeedPtr getVideoFeed();

    /**
     * Sets the aspect ratio of the widget and the feed.
     * The aspect ratio can only be set when no feed is registered
     * to the widgets.
     * This means before calling setVideoFeed(...)
     * or after calling releaseVideFeed().
     *
     * @param scaleX factor to multiply the feed width by.
     * @param scaleY factor to multiply the feed height by.
     *
     * @throws std::invalid_argument if either xScale or yScale is <= 0.
     * @throws std::runtime_error if a feed is registered to the widget.
     */
    void setScale(double scaleX, double scaleY);

    /**
     * Registers a VideoFeed to the widget.
     *
     * @throws  std::runtime_error if a VideoFeed is already registered.
     * @throws  std::invalid_argument if the VideoFeed is paused or playing.
     */
    void setVideoFeed(const GstVideoFeedPtr& feed);

    /**
     * Unregister the VideoFeed from the widget.
     * Does nothing if no VideoFeed has been registered.
     *
     * @throws  std::invalid_argument if the VideoFeed is paused or playing.
     */
    void releaseVideoFeed();

private:
    GstVideoFeedPtr mFeed;
    QGst::ElementPtr mVideoScale;
    QGst::ElementPtr mCapsFilter;
    QGst::ElementPtr mXImageSink;
    QGst::ElementPtr mVideoFlip;
    QGst::BinPtr mBin;

    double mScaleX;
    double mScaleY;

    void stopPipelineWatchTS();
    void watchPipelineTS(const QGst::PipelinePtr& pipeline);

private slots:
    void onStopPipelineWatch();
    void onWatchPipeline(const QGst::PipelinePtr& pipeline);
};

} /* namespace rimstreamer */

#endif /* RIMSTREAMER_VIDEOFEEDWIDGET_H_ */
