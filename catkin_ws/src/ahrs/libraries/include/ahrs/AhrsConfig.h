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

/**
 * \brief Configuration object passed to the factory of AHRS
 *
 * This object contains all the pertinent information to correctly instantiate an ahrs object. By default,
 * the object is given the following properties:
 *
 * \code
 * mDevName = "/dev/ahrs";
 * mBaudRate = 115200;
 * mSimulationMode = true;
 * \endcode
 */
class AhrsConfig
{
public:
    /**
     * \brief Constructor.
     *
     * Creates a default config object
     */
    AhrsConfig ();

    /**
     * \brief Destructor
     */
    virtual ~AhrsConfig ();

    /**
     * \brief Get the current baudrate set for this config
     *
     * \return The current baud rate in bauds per second
     */
    unsigned int getBaudRate() const;

    /**
     * \brief Set the baud rate parameter
     */
    void setBaudRate(unsigned int aBaudRate);

    /**
     * \brief Get the current device file pointer set for this config
     *
     * \return The current device pointer
     */
    std::string getDeviceName() const;

    /**
     * \brief Set the baud rate parameter
     */
    void setDeviceName(std::string aDeviceName);

    /**
     * \brief Get the current simulation flag set for this config
     *
     * \return The current simulation flag
     */
    bool isSimulation() const;

    /**
     * \brief Set the simulation parameter
     *
     * If the parameter is set to true, the ahrs created will not use the device name and baudrate, it will use random
     * number generators to populate the fields of the AhrsStatus structure returned in the getStatus method.
     */
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
