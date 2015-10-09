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

/**
 * \brief Implementation of the Ahrs interface specific to the SGB IG500-n device.
 *
 * Use the sbgCom library to realise the interface set forth by the Ahrs interface.
 */
class AhrsIg500n: public Ahrs{
public:
    /**
     * \brief Constructor initialises the sbgCom library and starts reception of updates from the device
     *
     * Registers appropriate normal and error callback functions for the IG500-n device.
     *
     * Creates a periodic thread to start parallel reception of the status updates.
     */
    AhrsIg500n(const char * deviceName, uint32 baudRate);

    /**
     * \brief Destructor.
     *
     * Close the communication with the device
     */
    virtual ~AhrsIg500n();

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

    /// Mutex for thread-safe operation
    mutable boost::mutex mMutex;

    /// Controller periodic thread
    boost::scoped_ptr<common::PeriodicThread> mpThread;

    /**
     * \brief Update internal data to latest info from device
     *
     * Method to be called by internal periodic thread. The method services the reception buffer and redirects to
     * the proper callback functions.
     *
     */
    void update();

    /// Handle for the sbg com library
    SbgProtocolHandle mProtocolHandle;
    /// Internal device file pointer
    const char * mDeviceName;
    /// Baud rate for serial communication
    uint32 mBaudRate;
    /// Holder for return value of sbc library functions
    SbgErrorCode mError;
    /// Internal status information structure for thread safe operation
    AhrsStatus mStatus;
    /// Buffer for the error messages received from the library
    char mErrorMsg[256];

    /**
     * \brief Handler for error flagged messages from the device
     *
     * This callback is called the the library receives a wrong frame from the ahrs
     */
    void continuousErrorCallback(SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode);

    /**
     * \brief Handler for normal operation
     *
     * This callback handles the correct frames received by the library,
     *
     * It updates the internal status structure and is thread safe.
     */
    void continuousCallback(SbgProtocolHandleInt *handler, SbgOutput *pOutput);

    /**
     * \brief Static wrapper to allow the use of the member error callback function
     */
    static void errorCallbackWrapper(SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode, void *pUsrArg);

    /**
     * \brief Static wrapper to allow the use of the member normal callback function
     */
    static void normalCallbackWrapper(SbgProtocolHandleInt *handler, SbgOutput *pOutput, void *pUsrArg);



};
}
}



#endif //LINERANGER_AHRS_IG500N_H
