//
// Created by David Lavoie-Boutin on 15-05-05.
//

#ifndef LINERANGER_AHRS_IG500N_H
#define LINERANGER_AHRS_IG500N_H

#include "sbgCom.h"
#include "PeriodicThread.h"

#include <boost/thread/mutex.hpp>
#include <boost/scoped_ptr.hpp>

#include <Ahrs.h>

#define UPDATE_FREQUENCY_MS 50
#define OUTPUT_DIVISOR 1

namespace lineranger
{
namespace ahrs
{
class AhrsIg500n: public Ahrs{
public:
    AhrsIg500n(const char * deviceName, uint32 baudRate);
    virtual ~AhrsIg500n();

    //-------------------------------------------------------------------------
    // AHRS implementation
    //-------------------------------------------------------------------------

    virtual AhrsStatus getStatus();

private:

    /// Mutex for thread-safe operation
    mutable boost::mutex mMutex;

    /// Controller periodic thread
    boost::scoped_ptr<common::PeriodicThread> mpThread;

    /// Method to be called by internal periodic thread
    void update();

    SbgProtocolHandle mProtocolHandle;
    const char * mDeviceName;
    uint32 mBaudRate;
    SbgErrorCode mError;
    SbgOutput mOutput;
    AhrsStatus mStatus;

    void continuousErrorCallback(SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode);
    void continuousCallback(SbgProtocolHandleInt *handler, SbgOutput *pOutput);
    char mErrorMsg[256];
    static void errorCallbackWrapper(SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode, void *pUsrArg);
    static void normalCallbackWrapper(SbgProtocolHandleInt *handler, SbgOutput *pOutput, void *pUsrArg);

};
}
}



#endif //LINERANGER_AHRS_IG500N_H
