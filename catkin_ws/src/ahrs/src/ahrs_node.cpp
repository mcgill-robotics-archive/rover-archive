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
#include <cstring>

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

        // Format headers
        std_msgs::Header header;
        header.frame_id = "ahrs";

        msg.gps.longitude = ahrsStatus.gpsLongitude / 10000000.0;
        msg.gps.altitude = ahrsStatus.gpsAltitude / 1000.0;
        msg.gps.latitude = ahrsStatus.gpsLatitude / 10000000.0;
        
        msg.gps.fix_type = ahrsStatus.gpsFlags & 0b11;

        msg.gps.validUTC = (ahrsStatus.gpsFlags && SBG_GPS_VALID_UTC) >> 4;

        msg.gps.velocity.linear.x = ahrsStatus.gpsVelocity[0] / 100.0;
        msg.gps.velocity.linear.y = ahrsStatus.gpsVelocity[1] / 100.0;
        msg.gps.velocity.linear.z = ahrsStatus.gpsVelocity[2] / 100.0;

        msg.gps.vertical_accuracy = ahrsStatus.gpsVertAccuracy / 100.0;
        msg.gps.horizontal_accuracy = ahrsStatus.gpsHoriAccuracy / 100.0;
        msg.gps.speed_accuracy = ahrsStatus.gpsSpeedAccuracy / 10.0;
        msg.gps.heading_accuracy = ahrsStatus.gpsHeadingAccuracy / 100000.0;

        msg.gps.heading = ahrsStatus.gpsHeading;

        msg.pose.header = header;
        msg.pose.pose.orientation.w = ahrsStatus.quaternion[0];
        msg.pose.pose.orientation.x = ahrsStatus.quaternion[1];
        msg.pose.pose.orientation.y = ahrsStatus.quaternion[2];
        msg.pose.pose.orientation.z = ahrsStatus.quaternion[3];

        msg.pose.pose.position.x = ahrsStatus.position[0];
        msg.pose.pose.position.y = ahrsStatus.position[1];
        msg.pose.pose.position.z = ahrsStatus.position[2];

        msg.attitude_accuracy = ahrsStatus.attitudeAccuracy;
        
        msg.gyroscopes.x = ahrsStatus.gyroscopes[0];
        msg.gyroscopes.y = ahrsStatus.gyroscopes[1];
        msg.gyroscopes.z = ahrsStatus.gyroscopes[2];

        msg.accelerometers.x = ahrsStatus.accelerometers[0];
        msg.accelerometers.y = ahrsStatus.accelerometers[1];
        msg.accelerometers.z = ahrsStatus.accelerometers[2];

        // memcpy(msg.accelerometers, ahrsStatus.accelerometers, sizeof(ahrsStatus.accelerometers));
        // memcpy(msg.gyroscopes, ahrsStatus.gyroscopes, sizeof(ahrsStatus.gyroscopes));

        ahrsPublisher.publish(msg);
        loopRate.sleep();
    }

    return 0;
}
