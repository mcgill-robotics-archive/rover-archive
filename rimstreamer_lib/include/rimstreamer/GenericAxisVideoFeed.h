/*
 * AbstractAxisM3007VideoFeed.h
 *
 *  Created on: 2016-02-29
 *      Author: cf0548
 */

#ifndef RIMSTREAMER_ABSTRACTAXISM3007VIDEOFEED_H_
#define RIMSTREAMER_ABSTRACTAXISM3007VIDEOFEED_H_

#include "rimstreamer/GstVideoFeed.h"

#include <QGst/Pad>
#include <QGst/Utils/ApplicationSink>

namespace rimstreamer
{

class GenericAxisVideoFeed: public GstVideoFeed
{
public:
    GenericAxisVideoFeed(const std::string &ipAddress, unsigned int cameraNo,
                         QSize &resolution, const std::string cameraName);

    virtual ~GenericAxisVideoFeed();

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
    virtual QString toString() const;

    void playAndRecord(const QString& filename);
    void stopPlayAndRecord();

private:
    QSize mResolution;
    QString mLabel;

    QGst::PipelinePtr mPipeline;
    QGst::ElementPtr mRtph264depay;

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

    void onRtspSrcPadAdded(const QGst::PadPtr& source);

    QGst::BinPtr createRecordBin(const QString& filename);
    QGst::BinPtr createViewBin();

    void activateViewPipeline();
    void deactivateViewPipeline();
    void activateRecordPipeline(const QString& filename);
    void deactivateRecordPipeline();
};

} /* namespace rimstreamer */

#endif /* RIMSTREAMER_ABSTRACTAXISM3007VIDEOFEED_H_ */
