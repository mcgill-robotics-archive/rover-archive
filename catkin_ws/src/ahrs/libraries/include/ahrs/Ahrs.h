#ifndef LINERANGER_AHRS_H
#define LINERANGER_AHRS_H

#include <stdint.h>
#include <time.h>
#include "AhrsConfig.h"

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
    /// Longitude in degrees (1e-7)
    int32_t gpsLongitude;
    /// Latitude in degrees (1e-7)
    int32_t gpsLatitude;
    /// Height above sea level in mm
    int32_t gpsAltitude;

    /// The pitch angle in radians from the euler angles
    float pitch;
    /// The roll angle in radians from the euler angles
    float roll;
    /// The yaw angle in radians from the euler angles
    float yaw;

    /// 3d velocity in device coordinate and in m/s
    float velocity[3];

    /// GPS True heading accuracy value. (1e-5 unit)
    int32_t heading;
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
