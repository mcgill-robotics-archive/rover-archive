/*
 * GstVideoFeed.h
 *
 *  Created on: May 17, 2016
 *      Author: cf0548
 */

#ifndef RIMSTREAMER_GSTVIDEOFEED_H_
#define RIMSTREAMER_GSTVIDEOFEED_H_

#include "rimstreamer/VideoFeed.h"

namespace rimstreamer
{

class GstVideoFeed: public VideoFeed
{
public:
    GstVideoFeed();
    virtual ~GstVideoFeed();

    virtual QGst::PipelinePtr getPipeline() = 0;
    virtual void releaseVideoSink() = 0;
    virtual void setVideoSink(QGst::ElementPtr sink) = 0;

    static QString getStateName(QGst::State val);
    static QString getStateChangeReturnName(QGst::StateChangeReturn val);
    static QString getStreamStatusName(QGst::StreamStatusType val);

    void onBusMessage(const QGst::MessagePtr& message);

protected:
    /**
     * Adds the element to the bin.
     *
     * @param bin bin to add the element to.
     * @param element element to add.
     *
     * @throw std::runtime_exception if the element could not be added
     *      to the bin.
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
    QGst::ElementPtr createElement(const QString& name,
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
     * Removes the element from the bin.
     *
     * @param bin bin to remove the element from.
     * @param element element to remove.
     */
    void removeElement(const QGst::BinPtr& bin,
            const QGst::ElementPtr& element);
};

typedef boost::shared_ptr<GstVideoFeed> GstVideoFeedPtr;

} /* namespace rimstreamer */

#endif /* RIMSTREAMER_GSTVIDEOFEED_H_ */
