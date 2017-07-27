/*
 * NullVideoFeed.h
 *
 *  Created on: 2016-04-04
 *      Author: rml00
 */

#ifndef RIMSTREAMER_NULLVIDEOFEED_H_
#define RIMSTREAMER_NULLVIDEOFEED_H_

#include "VideoFeed.h"

namespace rimstreamer
{

class NullVideoFeed: public VideoFeed
{
public:
    NullVideoFeed(int width, int height);
    virtual ~NullVideoFeed();

    bool contains(QGst::ElementPtr element) const;
    QGst::PipelinePtr getPipeline();
    QString getRecordFileExtension() const;
    const QSize& getResolution() const;
    QGst::State getState();
    bool isPlaying() const;
    bool isRecording() const;
    bool isStopped() const;
    void play();
    void releaseVideoSink();
    void setVideoSink(QGst::ElementPtr sink);
    void startRecord(const QString& filename);
    void stop();
    void stopRecord();
    void takeSnapshot(const QString& filename);
    QString toString() const;

    void playAndRecord(const QString& filename);
    void stopPlayAndRecord();

private:
    QSize mResolution;
    QGst::State mState;
    QGst::ElementPtr mVideoSink;

    bool mIsRecording;
};

} /* namespace rimstreamer */

#endif /* RIMSTREAMER_NULLVIDEOFEED_H_ */
