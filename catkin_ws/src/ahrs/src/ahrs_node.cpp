//
// Created by david on 7/21/15.
//

#include <Ahrs.h>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <signal.h>
#include <ahrs/AhrsStatusMessage.h>
#include "ros/ros.h"

void mySigintHandler(int sig)
{
    ROS_INFO("ahrs_node terminated");
    ros::shutdown();
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ahrs_node");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler);

    ROS_INFO("ahrs_node launching");

    lineranger::ahrs::AhrsConfig config;
    config.setSimulation(false);
    config.setDeviceName("/dev/ahrs");
    boost::scoped_ptr<lineranger::ahrs::Ahrs> ahrs;
    try
    {
        ahrs.reset(lineranger::ahrs::Ahrs::createAhrs(config));
    }
    catch (const std::runtime_error& error)
    {
        //ROS_ERROR(error.what());
        config.setSimulation(true);
        ROS_WARN("real ahrs not found, creating virtual ahrs");
        ahrs.reset(lineranger::ahrs::Ahrs::createAhrs(config));
    }

    ros::Publisher ahrsPublisher = nh.advertise<ahrs::AhrsStatusMessage>("ahrs_status", 100);
    lineranger::ahrs::AhrsStatus ahrsStatus;

    ros::Rate loopRate(10);
    ahrs::AhrsStatusMessage msg;
    ROS_INFO("ahrs_node ready, starting acquisition");
    while (ros::ok())
    {
        ahrsStatus = ahrs->getStatus();
        msg.gpsLongitude = ahrsStatus.gpsLongitude / 10000000.0;
        msg.gpsAltitude = ahrsStatus.gpsAltitude / 1000.0;
        msg.gpsLatitude = ahrsStatus.gpsLatitude / 10000000.0;

        msg.heading = ahrsStatus.heading;
        msg.pitch = ahrsStatus.pitch;
        msg.roll = ahrsStatus.roll;
        msg.yaw = ahrsStatus.yaw;

        msg.velocity.x = ahrsStatus.velocity[0];
        msg.velocity.y = ahrsStatus.velocity[1];
        msg.velocity.z = ahrsStatus.velocity[2];

        ahrsPublisher.publish(msg);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
