/*
 * TestVideoFeed.h
 *
 *  Created on: 2016-02-26
 *      Author: cf0548
 */

#ifndef RIMSTREAMER_TESTVIDEOFEED_H_
#define RIMSTREAMER_TESTVIDEOFEED_H_

#include "rimstreamer/GstVideoFeed.h"

namespace rimstreamer
{

typedef enum
{
    GST_VIDEO_TEST_SRC_SMPTE,
    GST_VIDEO_TEST_SRC_SNOW,
    GST_VIDEO_TEST_SRC_BLACK,
    GST_VIDEO_TEST_SRC_WHITE,
    GST_VIDEO_TEST_SRC_RED,
    GST_VIDEO_TEST_SRC_GREEN,
    GST_VIDEO_TEST_SRC_BLUE,
    GST_VIDEO_TEST_SRC_CHECKERS1,
    GST_VIDEO_TEST_SRC_CHECKERS2,
    GST_VIDEO_TEST_SRC_CHECKERS4,
    GST_VIDEO_TEST_SRC_CHECKERS8,
    GST_VIDEO_TEST_SRC_CIRCULAR,
    GST_VIDEO_TEST_SRC_BLINK,
    GST_VIDEO_TEST_SRC_SMPTE75,
    GST_VIDEO_TEST_SRC_ZONE_PLATE,
    GST_VIDEO_TEST_SRC_GAMUT,
    GST_VIDEO_TEST_SRC_CHROMA_ZONE_PLATE,
    GST_VIDEO_TEST_SRC_SOLID,
    GST_VIDEO_TEST_SRC_BALL,
    GST_VIDEO_TEST_SRC_SMPTE100,
    GST_VIDEO_TEST_SRC_BAR,
    GST_VIDEO_TEST_SRC_PINWHEEL,
    GST_VIDEO_TEST_SRC_SPOKES
} GstVideoTestSrcPattern;

class TestVideoFeed: public GstVideoFeed
{
public:
    TestVideoFeed(unsigned int width = 320, unsigned int height = 240,
            GstVideoTestSrcPattern pattern = GST_VIDEO_TEST_SRC_SMPTE);
    virtual ~TestVideoFeed();

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
    QGst::PipelinePtr mPipeline;
    QGst::ElementPtr mViewSrc;
    QGst::ElementPtr mViewSink;

    bool mIsRecording;

    QSize mResolution;
    GstVideoTestSrcPattern mPattern;
};

} /* namespace rimstreamer */

#endif /* RIMSTREAMER_TESTVIDEOFEED_H_ */
