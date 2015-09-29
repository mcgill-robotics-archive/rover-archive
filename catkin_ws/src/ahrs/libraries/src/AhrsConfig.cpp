/*
 * AhrsConfig.cpp
 *
 *  Created on: May 8, 2015
 *      Author: david
 */

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <AhrsConfig.h>

using namespace lineranger:: ahrs;

AhrsConfig::AhrsConfig ()
{
    mDevName = "/dev/ahrs";
    mBaudRate = 115200;
    mSimulationMode = true;
}

AhrsConfig::~AhrsConfig ()
{

}

unsigned int AhrsConfig::getBaudRate() const
{
    return mBaudRate;
}
void AhrsConfig::setBaudRate(unsigned int aBaudRate)
{
    mBaudRate = aBaudRate;
}
std::string AhrsConfig::getDeviceName() const
{
    return mDevName;
}
void AhrsConfig::setDeviceName(std::string aDeviceName)
{
    mDevName = aDeviceName;
}

bool AhrsConfig::isSimulation() const
{
    return mSimulationMode;
}
void AhrsConfig::setSimulation(bool isSimulation)
{
    mSimulationMode = isSimulation;
}
