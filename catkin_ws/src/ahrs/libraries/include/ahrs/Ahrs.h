#ifndef LINERANGER_AHRS_H
#define LINERANGER_AHRS_H

#include <stdint.h>
#include <time.h>
#include "AhrsConfig.h"
#include <sbgCom.h>

namespace lineranger
{
namespace ahrs
{

/**
 * \brief AHRS status components
 *
 * This structure collects all the data output from the AHRS device.
 */
typedef struct _AhrsStatus {
 
    /// Angle representation in quaternions
    float quaternion[4];   
    
    /// Longitude in degrees (1e-7)
    int32_t gpsLongitude;
    /// Latitude in degrees (1e-7)
    int32_t gpsLatitude;
    /// Height above sea level in mm
    int32_t gpsAltitude;

    /// Horizontal accuracy in mm
    uint32_t gpsHoriAccuracy;
    /// Vertical accuracy in mm
    uint32_t gpsVertAccuracy;
 
    /// 3d velocity cm/s in north east down coordinate
    int32_t gpsVelocity[3];

    uint8_t gpsFlags;
    uint8_t gpsNbSat;

    /// GPS True heading value. (1e-5 unit)
    int32_t gpsTrueHeading;
    /// 2D GPS computed heading in degrees
    int32_t gpsHeading;

    uint32_t deviceStatus;

    /// position in NED coordinates in (degrees, degrees, meters)
    double position[3];
    /// 3d velocity in device coordinate and in m/s
    float velocity[3];

} AhrsStatus;


/**
 * \brief Interface to the ahrs library
 *
 * Point of entry is the createAhrs static method.
 */
class Ahrs {
public:

    /**
     * \brief Pure virtual method that needs to be implemented by each implementation of this interface
     *
     * \return Returns a structure contaiting all the pertinant information provided by the device
     */
    virtual AhrsStatus getStatus() = 0;

    /**
     * \brief Point of entry to the interface
     *
     * This method instantiates the proper implementation of the ahrs based on the configuration object received
     *
     * Returns a raw pointer to an AHRS object. This pointer must be properly managed to avoid memory leaks.
     */
    static Ahrs* createAhrs(const AhrsConfig& config);

    /**
     * \brief Destructor.
     */
    virtual ~Ahrs();

protected:

    Ahrs();

};


} // ahrs
} // lineranger


#endif //LINERANGER_AHRS_H
