//
// Created by David Lavoie-Boutin on 2016-01-31.
//

#ifndef ROVER_ARDUINO_TRANSFORMSENDER_H
#define ROVER_ARDUINO_TRANSFORMSENDER_H

#include <ros.h>
#include <tf/transform_broadcaster.h>
#include "TransformConfig.h"

namespace arm {


/**
 * This class wants to wrap the mechanics of sending transforms for the rover
 * 6 DOF arm. 
 */
class TransformSender {
public:
    TransformSender(ros::NodeHandle &nh, TransformConfig &config);
    void init();
    void updateRotations(float yaw_base, float pitch_base, float pitch2, float roll1, float pitch3, float roll2);
    void sendTransforms();
    virtual ~TransformSender();

private:
    geometry_msgs::TransformStamped baseYaw;
    geometry_msgs::TransformStamped mPitch1;
    geometry_msgs::TransformStamped mPitch2;
    geometry_msgs::TransformStamped mRoll1;
    geometry_msgs::TransformStamped mPitch3;
    geometry_msgs::TransformStamped mRoll2;

    tf::TransformBroadcaster broadcaster;

    void from_euler(float roll, float pitch, float yaw, geometry_msgs::Quaternion & quaternion);
    ros::NodeHandle * mNh;
    ros::Time mTime;
};
}

#endif //ROVER_ARDUINO_TRANSFORMSENDER_H
