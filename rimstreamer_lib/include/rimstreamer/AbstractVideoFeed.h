/*
 * AbstractVideoFeed.h
 *
 *  Created on: 2016-04-04
 *      Author: rml00
 */

#ifndef RIMSTREAMER_ABSTRACTVIDEOFEED_H_
#define RIMSTREAMER_ABSTRACTVIDEOFEED_H_

#include "rimstreamer/VideoFeed.h"

#include <QGst/Pipeline>

namespace rimstreamer
{

class AbstractVideoFeed: public VideoFeed
{
public:
    AbstractVideoFeed();
    virtual ~AbstractVideoFeed();

public:

    bool contains(QGst::ElementPtr element) const;
    QGst::PipelinePtr getPipeline();
    QGst::State getState();
    bool isPaused() const;
    bool isPlaying() const;
    bool isStopped() const;
    void pause();
    void play();
    void releaseVideoSink();
    void setVideoSink(QGst::ElementPtr sink);
    void stop();

protected:

    /**
     * Adds the element to the pipeline.
     *
     * @param element element to add.
     *
     * @throw std::runtime_exception if the element could not be added
     *      to the pipeline.
     */
    void addElement(const QGst::ElementPtr& element);

    /**
     * Adds the element to the bin.
     *
     * @param bin bin to add the element to.
     * @param element element to add.
     *
     * @throw std::runtime_exception if the element could not be added
     *      to the pipeline.
     */
    void addElement(const QGst::BinPtr& bin, const QGst::ElementPtr& element);

    /**
     * Adds a ghost pad from the bin to the element.
     *
     * @param bin bin to add the ghost pad to.
     * @param element element to link the ghost pad to.
     * @param name name of the element's pad to link to the ghost pad.
     *             The ghost pad will have the same name.
     */
    void addGhostPad(const QGst::BinPtr& bin, const QGst::ElementPtr& element,
            const std::string& name);

    /**
     * Creates a 'capsfilter' element.
     *
     * @param filter the capsfilter arguments
     *
     * @return a smart pointer to the capsfilter element.
     */
    QGst::ElementPtr createCapsFilter(const QString& filter);

    /**
     * Creates a new element.
     *
     * @param name name of the element (ex.: videotestsrc).
     * @param properties hash map of the elements properties.
     *
     * @return A smart pointer to the created element.
     *
     * @throw std::runtime_exception if the element could not be created.
     */
    static QGst::ElementPtr createElement(const QString& name,
            const QHash<QString, QGlib::Value>& properties = QHash<QString,
                    QGlib::Value>());

    /**
     * Links the source to the sink.
     *
     * @param source source to link from.
     * @param sink sink to link to.
     *
     * @throw std::runtime_exception if the link fails.
     *      or if the source or sink has not been previously added to the
     *      pipeline.
     */
    void link(const QGst::ElementPtr& source, const QGst::ElementPtr& sink);

    /**
     * Links the source to the sink in the bin.
     *
     * @param bin bin holding the elements to link.
     * @param source source to link from.
     * @param sink sink to link to.
     */
    void link(const QGst::BinPtr& bin, const QGst::ElementPtr& source,
            const QGst::ElementPtr& sink);

    /**
     * Set the source that will be connected to the final video sink.
     * The source must first have been added to the pipeline.
     *
     * @param source the source
     *
     * @throw std::runtime_exception if the source has not been previously added
     *      to the pipeline.
     */
    void setVideoSource(QGst::ElementPtr source);

    /**
     * Removes the element from the pipeline.
     * Does nothing if the element is not in the pipeline.
     *
     * @param element element to remove.
     */
    void removeElement(QGst::ElementPtr element);

private:
    QGst::PipelinePtr mPipeline;
    std::vector<QGst::ElementPtr> mElements;
    QGst::ElementPtr mVideoSource;
    QGst::ElementPtr mVideoSink;

    QString getStateChangeReturnName(QGst::StateChangeReturn val);
    void onBusSyncMessage(const QGst::MessagePtr& message);
};

} /* namespace rimstreamer */

#endif /* RIMSTREAMER_ABSTRACTVIDEOFEED_H_ */
