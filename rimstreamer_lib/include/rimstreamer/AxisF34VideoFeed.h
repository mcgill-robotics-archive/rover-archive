/*
 * AxisF34VideoFeed.h
 *
 *  Created on: 2016-03-29
 *      Author: cf0548
 */

#ifndef RIMSTREAMER_AXISF34VIDEOFEED_H_
#define RIMSTREAMER_AXISF34VIDEOFEED_H_

#include "rimstreamer/GstVideoFeed.h"
#include "GenericAxisVideoFeed.h"

#include <QGst/Pad>
#include <QGst/Utils/ApplicationSink>

namespace rimstreamer
{
enum AxisF34View_t {
    F34_CAMERA_1 = 1,
    F34_CAMERA_2 = 2,
    F34_CAMERA_3 = 3,
    F34_CAMERA_4 = 4,
    F34_QUAD = 99
};

enum AxisF34Resolution_t {
    F34_1024x768 = 0,
    F34_800x600 = 1,
    F34_640x480 = 2,
    F34_480x360 = 3,
    F34_1280x720 = 4,
    F34_854x480 = 5,
    F34_800x450 = 6,
    F34_640x360 = 7,
    F34_480x270 = 8,
    F34_1024x640 = 9,
    F34_800x500 = 10,
    F34_640x400 = 11,
    F34_480x300 = 12
};

class AxisF34VideoFeed : public GenericAxisVideoFeed {
public:
    AxisF34VideoFeed(const std::string& ipAddress, AxisF34View_t cameraNo, AxisF34Resolution_t resolution, Orientation_t orientation);
    virtual ~AxisF34VideoFeed();

private:
    static const unsigned short RESOLUTION_COUNT = 24;
    static QSize mResolutions[RESOLUTION_COUNT];
    static QSize& getResolution(AxisF34Resolution_t index);
};

} /* namespace rimstreamer */

#endif /* RIMSTREAMER_AXISF34VIDEOFEED_H_ */
