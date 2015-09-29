//
// Created by david on 05/05/15.
//

#include <Ahrs.h>

#include "AhrsIg500n.h"
#include "AhrsVirtual.h"


using namespace lineranger;
using namespace lineranger::ahrs;

Ahrs::Ahrs() { }
Ahrs::~Ahrs() { }

Ahrs* Ahrs::createAhrs(const AhrsConfig& config) {
    if (config.isSimulation()){

        return new AhrsVirtual();
    }
    else
    {
        return new AhrsIg500n(config.getDeviceName().c_str(), config.getBaudRate());
    }



}
