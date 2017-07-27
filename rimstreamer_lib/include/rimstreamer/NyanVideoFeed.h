/*
 * NyanVideoFeed.h
 *
 *  Created on: 2016-03-06
 *      Author: cf0548
 */

#ifndef RIMSTREAMER_NYANVIDEOFEED_H_
#define RIMSTREAMER_NYANVIDEOFEED_H_

#include "rimstreamer/GstVideoFeed.h"

#include <QFileInfo>
#include <QGst/Utils/ApplicationSink>

namespace rimstreamer
{

enum NyanType_t
{
    CAT, DOG
};

class NyanVideoFeed: public GstVideoFeed
{
public:
    NyanVideoFeed(NyanType_t type, bool flip = false, bool invert = false);
    virtual ~NyanVideoFeed();

    QGst::PipelinePtr getPipeline();
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

    QString getRecordFileExtension() const;
    const QSize& getResolution() const;
    QString toString() const;

    void playAndRecord(const QString& filename);
    void stopPlayAndRecord();

private:
    NyanType_t mType;
    bool mFlip;
    bool mInvert;
    QFileInfo mLocation;
    QSize mResolution;

    QGst::PipelinePtr mPipeline;
    QGst::ElementPtr mH264parse;

    QGst::ElementPtr mQueueView;
    QGst::BinPtr mViewBin;
    QGst::ElementPtr mFakeViewSink;
    QGst::ElementPtr mViewSink;

    QGst::Utils::ApplicationSink mAppSink;

    QGst::ElementPtr mQueueRecord;
    QGst::BinPtr mRecordBin;
    QGst::ElementPtr mFakeRecordSink;

    bool mIsPaused;
    bool mIsPlaying;
    bool mIsRecording;

    void onQtDemuxPadAdded(const QGst::PadPtr& pad);

    QGst::BinPtr createRecordBin(const QString& filename);
    QGst::BinPtr createViewBin(bool flip, bool invert);

    void activateViewPipeline();
    void deactivateViewPipeline();
    void activateRecordPipeline(const QString& filename);
    void deactivateRecordPipeline();
};

} /* namespace rimstreamer */

#endif /* RIMSTREAMER_NYANVIDEOFEED_H_ */
