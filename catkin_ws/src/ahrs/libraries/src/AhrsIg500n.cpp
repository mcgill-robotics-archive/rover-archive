//
// Created by David Lavoie-Boutin on 15-05-05.
//

#include "AhrsIg500n.h"
#include "stdio.h"
#include "ros/ros.h"


using namespace lineranger::ahrs;

AhrsIg500n::AhrsIg500n(const char * deviceName, uint32 baudRate) {
    mDeviceName = deviceName;
    mBaudRate = baudRate;

    mError = sbgComInit(mDeviceName, mBaudRate, &mProtocolHandle);
        if (mError != SBG_NO_ERROR) {
        sbgComErrorToString(mError, mErrorMsg);
        std::runtime_error e(mErrorMsg);
        //ROS_ERROR(e.what());
        throw e;
    }

    mError = sbgSetDefaultOutputMask(mProtocolHandle, SBG_OUTPUT_EULER |
                                                      SBG_OUTPUT_GPS_POSITION |
                                                      SBG_OUTPUT_VELOCITY |
                                                      SBG_OUTPUT_GPS_TRUE_HEADING);
    if (mError != SBG_NO_ERROR) {
        sbgComErrorToString(mError, mErrorMsg);
        std::runtime_error e(mErrorMsg);
        //ROS_ERROR(e.what());
    }
    mError = sbgSetContinuousMode(mProtocolHandle, SBG_CONTINUOUS_MODE_ENABLE, OUTPUT_DIVISOR);
    if (mError != SBG_NO_ERROR) {
        sbgComErrorToString(mError, mErrorMsg);
        std::runtime_error e(mErrorMsg);
        //ROS_ERROR(e.what());
    }
    sbgSetContinuousErrorCallback(mProtocolHandle, &errorCallbackWrapper, (void *)(this));
    sbgSetContinuousModeCallback(mProtocolHandle, &normalCallbackWrapper, (void *)(this));

    // Spawn periodic thread
    printf("Creating thread\n");
    mpThread.reset( new common::PeriodicThread(
                    boost::bind(&AhrsIg500n::update, this),
                    UPDATE_FREQUENCY_MS, 0,
                    common::PeriodicThread::getAboveNormalPriority())); //TODO: Confirm priority value we want for thread

    if (!mpThread)
    {
        std::runtime_error e("Failed to create ahrs periodic thread.");
        //ROS_ERROR(e.what());
        throw e;
    }
    printf("Thread done\n");
}

AhrsIg500n::~AhrsIg500n() {
    sbgProtocolClose(mProtocolHandle);
}

AhrsStatus AhrsIg500n::getStatus() {
    boost::mutex::scoped_lock lock(mMutex);
    AhrsStatus status = mStatus;
    return status;

}

void AhrsIg500n::update() {

    // read buffer
    // lock
    SbgErrorCode error = sbgProtocolContinuousModeHandle(mProtocolHandle);
    if (error != SBG_NO_ERROR){
        char errorMsg[256];
        sbgComErrorToString(mError, errorMsg);
        std::runtime_error e(errorMsg);
        //ROS_ERROR(e.what());
    }
}
void AhrsIg500n::errorCallbackWrapper(SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode, void *pUsrArg){
    static_cast<AhrsIg500n*>(pUsrArg)->continuousErrorCallback(pHandler, errorCode);

}
void AhrsIg500n::normalCallbackWrapper(SbgProtocolHandleInt *handler, SbgOutput *pOutput, void *pUsrArg){
    static_cast<AhrsIg500n*>(pUsrArg)->continuousCallback(handler, pOutput);
}



void AhrsIg500n::continuousErrorCallback(SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode){
    boost::mutex::scoped_lock lock(mMutex);
    mError = errorCode;
    lock.unlock();

    sbgComErrorToString(mError, mErrorMsg);
    std::runtime_error e(mErrorMsg);
    //ROS_ERROR(e.what());
    return ;
}

void AhrsIg500n::continuousCallback(SbgProtocolHandleInt *handler, SbgOutput *pOutput){

    boost::mutex::scoped_lock lock(mMutex);AhrsStatus status;
    mStatus.gpsAltitude = pOutput->gpsAltitude;
    mStatus.gpsLatitude = pOutput->gpsLatitude;
    mStatus.gpsLongitude = pOutput->gpsLongitude;
    mStatus.heading = pOutput->gpsTrueHeading;
    mStatus.roll = pOutput->stateEuler[0];
    mStatus.pitch = pOutput->stateEuler[1];
    mStatus.yaw = pOutput->stateEuler[2];
    memcpy(mStatus.velocity, pOutput->velocity, 3);

}
