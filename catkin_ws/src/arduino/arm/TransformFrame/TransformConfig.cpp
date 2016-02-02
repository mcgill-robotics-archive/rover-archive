//
// Created by David Lavoie-Boutin on 2016-01-31.
//

#include "TransformConfig.h"

arm::TransformConfig::TransformConfig() {
    baseFrame = "/robot";
    baseYawFrame = "/arm/base_yaw";
    pitch1Frame = "/arm/base_pitch";
    pitch2Frame = "/arm/pitch2";
    roll1Frame = "/arm/roll1";
    pitch3Frame = "/arm/pitch3";
    roll2Frame = "/arm/roll2";
}
