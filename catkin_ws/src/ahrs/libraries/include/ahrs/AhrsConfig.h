/*
 * AhrsConfig.h
 *
 *  Created on: May 8, 2015
 *      Author: david
 */

#ifndef LIBS_AHRS_INCLUDE_AHRS_AHRSCONFIG_H_
#define LIBS_AHRS_INCLUDE_AHRS_AHRSCONFIG_H_

#include <string>

namespace lineranger
{
namespace ahrs
{

class AhrsConfig
{
public:
    AhrsConfig ();
    virtual ~AhrsConfig ();

    unsigned int getBaudRate() const;
    void setBaudRate(unsigned int aBaudRate);
    std::string getDeviceName() const;
    void setDeviceName(std::string aDeviceName);
    bool isSimulation() const;
    void setSimulation(bool isSimulation);

private:

    /// Serial communication baud rate (default is 115 200)
    unsigned int mBaudRate;

    /// Terminal device location (normally looks like "/dev/ttyUSB0")
    std::string mDevName;

    /// True if simulation mode enabled
    bool mSimulationMode;
};

} /* namespace ahrs */
} /* namespace lineranger */

#endif /* LIBS_AHRS_INCLUDE_AHRS_AHRSCONFIG_H_ */
