/*
 * VideoFeed.h
 *
 *  Created on: 2016-02-19
 *      Author: cf0548
 */

#ifndef RIMSTREAMER_VIDEOFEED_H_
#define RIMSTREAMER_VIDEOFEED_H_

#include <QHash>
#include <QSize>
#include <QString>
#include <QGlib/RefPointer>
#include <QGlib/Value>
#include <QGst/Element>
#include <QGst/Message>
#include <QGst/Pipeline>

#include <boost/shared_ptr.hpp>

#include <vector>

namespace rimstreamer
{

class VideoFeed: public QObject
{
public:
    VideoFeed();
    virtual ~VideoFeed();

public:

    /**
     * Returns the extension of the file format of the recording.
     * The extension returned does not have the leading dot character
     * (ex.: "mp4").
     *
     * @return the extension.
     */
    virtual QString getRecordFileExtension() const = 0;

    /**
     * Returns the resolution of the feed.
     *
     * @return the resolution of the feed.
     */
    virtual const QSize& getResolution() const = 0;

    /**
     * Checks if the feed is playing.
     */
    virtual bool isPlaying() const = 0;

    /**
     * Checks if the feed is recording.
     */
    virtual bool isRecording() const = 0;

    /**
     * Checks if the feed is stopped.
     */
    virtual bool isStopped() const = 0;

    /**
     * Will play the feed.
     * Does nothing if the feed is already playing.
     */
    virtual void play() = 0;

    /**
     * Starts playing and recording the feed.
     * Same as calling the play and playRecord method successively.
     *
     * @param filename file to save the recording to.
     */
    virtual void playAndRecord(const QString& filename) = 0;

    /**
     * Starts recording the feed.
     * The feed should be playing before the recording is started.
     *
     * @param filename file to save the recording to.
     */
    virtual void startRecord(const QString& filename) = 0;

    /**
     * Will stop the feed.
     * Does nothing if the feed is already stopped.
     */
    virtual void stop() = 0;

    /**
     * Stops playing and recording the feed.
     * Same as calling the stop and stopRecord method successively.
     */
    virtual void stopPlayAndRecord() = 0;

    /**
     * Stops the recording of the feed.
     * Does nothing if the feed is not being recorded.
     */
    virtual void stopRecord() = 0;

    /**
     * Takes a snapshot.
     *
     * @param filename file to put the snapshot in.
     */
    virtual void takeSnapshot(const QString& filename) = 0;

    /**
     * Returns a string representation of the feed.
     *
     * \return a string representation of the feed.
     */
    virtual QString toString() const = 0;
};

QDebug operator<<(QDebug debug, const VideoFeed& feed);

typedef boost::shared_ptr<VideoFeed> VideoFeedPtr;

} /* namespace rimstreamer */

#endif /* RIMSTREAMER_VIDEOFEED_H_ */
