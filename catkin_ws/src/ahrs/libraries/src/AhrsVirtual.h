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

class AhrsVirtual: public Ahrs {
public:
    AhrsVirtual();
    virtual ~AhrsVirtual();

    virtual AhrsStatus getStatus();

private:
    AhrsStatus mStatus;

    /// Mutex for thread-safe operation
    mutable boost::mutex mMutex;

    /// Controller periodic thread
    boost::scoped_ptr<common::PeriodicThread> mpThread;

    /// Method to be called by internal periodic thread
    void update();


};

}
}
#endif /* LIBS_AHRS_SRC_AHRSVIRTUAL_H_ */
