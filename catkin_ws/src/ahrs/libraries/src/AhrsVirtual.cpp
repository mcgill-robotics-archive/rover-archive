/*
 * AhrsVirtual.cpp
 *
 *  Created on: May 7, 2015
 *      Author: david
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include "AhrsVirtual.h"

using namespace lineranger::ahrs;
static const int UPDATE_FREQUENCY_MS = 100;
AhrsVirtual::AhrsVirtual()
{
    mpThread.reset( new common::PeriodicThread(
                        boost::bind(&AhrsVirtual::update, this),
                        UPDATE_FREQUENCY_MS, 0,
                        common::PeriodicThread::getAboveNormalPriority())); //TODO: Confirm priority value we want for thread

        if (!mpThread)
        {
            std::runtime_error e("Failed to create virtual ahrs periodic thread.");
//            LR_LOG_FATAL() << e.what();
            throw e;
        }

}

AhrsVirtual::~AhrsVirtual()
{

}

AhrsStatus AhrsVirtual::getStatus()
{
    AhrsStatus status;
    {
        boost::mutex::scoped_lock lock(mMutex);
        status = mStatus;
    }
    return status;
}


void AhrsVirtual::update()
{
    boost::mutex::scoped_lock lock(mMutex);

    mStatus.heading = (int32_t) (-M_PI + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2*M_PI))));

    mStatus.roll = (float) (-M_PI + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2*M_PI))));
    mStatus.yaw = (float) (-M_PI + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2*M_PI))));
    mStatus.pitch = (float) (-M_PI_2 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(M_PI))));

    mStatus.gpsAltitude = rand() /(RAND_MAX/(5000));
    mStatus.gpsLatitude = -90 * 10000 + rand() /(RAND_MAX/(180 * 10000));
    mStatus.gpsLongitude = -180 * 10000 + rand() /(RAND_MAX/(360 * 10000));

    mStatus.velocity[0] = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(100)));
    mStatus.velocity[1] = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(100)));
    mStatus.velocity[2] = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(100)));

}
