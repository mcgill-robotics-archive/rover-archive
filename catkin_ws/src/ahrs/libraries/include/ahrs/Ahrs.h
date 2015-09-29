//
// Created by David Lavoie-Boutin on 05/05/15.
//

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

class Ahrs {
public:

    virtual AhrsStatus getStatus() = 0;
    static Ahrs* createAhrs(const AhrsConfig& config);
    virtual ~Ahrs();
protected:

    Ahrs();

};


} // ahrs
} // lineranger


#endif //LINERANGER_AHRS_H
