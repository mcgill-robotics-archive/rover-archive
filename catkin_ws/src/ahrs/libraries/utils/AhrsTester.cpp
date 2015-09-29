/*
 * AhrsTester.cpp
 *
 *  Created on: May 6, 2015
 *      Author: David Lavoie-Boutin
 */

#include "ahrs/Ahrs.h"
#include "ahrs/AhrsConfig.h"
#include "stdio.h"
#include <common/Logging.h>
#include <common/Application.h>
#include <boost/scoped_ptr.hpp>

using namespace lineranger;

class AhrsConfGen : public common::Application
{
public:
    AhrsConfGen() :
        common::Application("conf-gen", "Ahrs Tester Configuration Generator", "")
    {

    }
    void addOptionsHook(po::options_description& options)
        {
            options.add_options()
                    ("port", po::value<std::string>()->default_value("/dev/ttyUSB0"), "the location of the terminal device (defaults to /dev/ttyUSB0)")
                    ("baud", po::value<int>()->default_value(115200), "serial communication baud rate (defaults at 115 200 baud)")
                    ("real", po::value<bool>()->default_value(false), "register the ahrs as a real device (defaults to false)");
        }

    bool runHook(const po::variables_map& args)
        {
            ahrs::AhrsConfig ahrsConfig;

            if(args.size() == 0)
            {
                printHelp();
            }
            else
            {

                if (args.count("port"))
                {
                    ahrsConfig.setDeviceName(args["port"].as<std::string>());
                }

                if (args.count("baud"))
                {
                    ahrsConfig.setBaudRate(args["baud"].as<int>());
                }
                if (args.count("real"))
                {
                    ahrsConfig.setSimulation(!args["real"].as<bool>());
                }
            }

            boost::scoped_ptr<lineranger::ahrs::Ahrs> ahrs(lineranger::ahrs::Ahrs::createAhrs(ahrsConfig));

            while (1){
                lineranger::ahrs::AhrsStatus status = ahrs->getStatus();

                printf("Euler: roll %f, pitch %f, yaw %f,  GPS: %d, %d, %d,  velocities: %f, %f, %f\n",
                        status.roll, status.pitch, status.yaw,
                        status.gpsAltitude, status.gpsLongitude, status.gpsLatitude,
                        status.velocity[0], status.velocity[1], status.velocity[2]);
                usleep(10000);
            }


            return true;
        }
};

int main (int argc, char** argv)
{
    AhrsConfGen app;
    return app.exec(argc, argv);
}
