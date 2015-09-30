/*
 * AhrsVirtual.h
 *
 *  Created on: May 7, 2015
 *      Author: david
 */

#ifndef LIBS_AHRS_SRC_AHRSVIRTUAL_H_
#define LIBS_AHRS_SRC_AHRSVIRTUAL_H_


#include <boost/thread/mutex.hpp>
#include <boost/scoped_ptr.hpp>
#include <Ahrs.h>
#include "PeriodicThread.h"

namespace lineranger
{
namespace ahrs
{
/**
 * \brief Implementation of the Ahrs interface with random data generation instead of physical device.
 *
 * This class add the option to generate virtual devices that provide dummy data to test the system from start to end.
 */
class AhrsVirtual: public Ahrs {
public:

    /**
     * \brief Constructor starts execution creation of random statuses
     *
     * Creates a periodic thread to start parallel call to the status updates.
     */
    AhrsVirtual();

    /**
     * \brief Destructor.
     */
    virtual ~AhrsVirtual();

    /**
     * \brief Realisation of the interface
     *
     * Implementation of the get status method.
     *
     * Return the latest information in a thread safe manner.
     *
     * \return Structure containing the most up-to-date position and orientation information received from the device.
     */
    virtual AhrsStatus getStatus();

private:
    AhrsStatus mStatus;

    /// Mutex for thread-safe operation
    mutable boost::mutex mMutex;

    /// Controller periodic thread
    boost::scoped_ptr<common::PeriodicThread> mpThread;

    /**
     * \brief Update the internal structure with some random values
     *
     * Method to be called by internal periodic thread
     */
    void update();


};

}
}
#endif /* LIBS_AHRS_SRC_AHRSVIRTUAL_H_ */
