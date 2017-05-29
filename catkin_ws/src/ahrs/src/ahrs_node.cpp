//
// Created by david on 7/21/15.
//

#include <Ahrs.h>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <signal.h>
#include <ahrs/AhrsStdMsg.h>
#include "ros/ros.h"
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Header.h>

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
        ROS_FATAL("real ahrs not found, exiting");
        return 1;
    }

    ros::Publisher ahrsPublisher = nh.advertise<ahrs::AhrsStdMsg>("ahrs_status", 100);
    lineranger::ahrs::AhrsStatus ahrsStatus;

    ros::Rate loopRate(10);
    ahrs::AhrsStdMsg msg;
    
    ROS_INFO("ahrs_node ready, starting acquisition");
    while (ros::ok())
    {
        ahrsStatus = ahrs->getStatus();

        std_msgs::Header header;

        header.frame_id = "ahrs";
        msg.pose.header = header;
        msg.gpsVelocity.header = header;
        msg.filteredVelocity.header = header;

        msg.gps.longitude = ahrsStatus.gpsLongitude / 10000000.0;
        msg.gps.altitude = ahrsStatus.gpsAltitude / 1000.0;
        msg.gps.latitude = ahrsStatus.gpsLatitude / 10000000.0;
        msg.gps.vertAccuracy = ahrsStatus.gpsVertAccuracy;
        msg.gps.horiAccuracy = ahrsStatus.gpsHoriAccuracy;

        int fix = ahrsStatus.gpsFlags & 0b11;
        msg.gps.FIX_3D = (fix == SBG_GPS_3D_FIX);
        msg.gps.FIX_2D = (fix == SBG_GPS_2D_FIX);
        msg.gps.NO_FIX = (fix == SBG_GPS_NO_FIX);
        msg.gps.TIME_FIX_ONLY = (fix == SBG_GPS_TIME_ONLY);

        msg.gps.validTOW = (ahrsStatus.gpsFlags && SBG_GPS_VALID_TOW) >> 2;
        msg.gps.validWKN = (ahrsStatus.gpsFlags && SBG_GPS_VALID_WKN) >> 3;
        msg.gps.validUTC = (ahrsStatus.gpsFlags && SBG_GPS_VALID_UTC) >> 4;


        msg.gpsVelocity.twist.linear.x = ahrsStatus.gpsVelocity[0];
        msg.gpsVelocity.twist.linear.y = ahrsStatus.gpsVelocity[1];
        msg.gpsVelocity.twist.linear.z = ahrsStatus.gpsVelocity[2];

        msg.filteredVelocity.twist.linear.x = ahrsStatus.velocity[0];
        msg.filteredVelocity.twist.linear.y = ahrsStatus.velocity[1];
        msg.filteredVelocity.twist.linear.z = ahrsStatus.velocity[2];

        msg.pose.pose.orientation.w = ahrsStatus.quaternion[0];
        msg.pose.pose.orientation.x = ahrsStatus.quaternion[1];
        msg.pose.pose.orientation.y = ahrsStatus.quaternion[2];
        msg.pose.pose.orientation.z = ahrsStatus.quaternion[3];

        msg.pose.pose.position.x = ahrsStatus.position[0];
        msg.pose.pose.position.y = ahrsStatus.position[1];
        msg.pose.pose.position.z = ahrsStatus.position[2];

        msg.gpsHeading.data = ahrsStatus.gpsHeading;

        ahrsPublisher.publish(msg);
        loopRate.sleep();
    }

    return 0;
}
