//
// Created by David Lavoie-Boutin on 25/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <ros/node_handle.h>
#include "NavigationController.h"

const double rad2deg = 180 / 3.141592653;

NavigationController::NavigationController(ros::NodeHandle &nh) : mNh(nh) {

}

void NavigationController::ahrs_ros_cb(const ahrs::AhrsStdMsg &ahrsMsg) {
    tf::quaternionMsgToTF(ahrsMsg.pose.pose.orientation, quat);
    tf::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);

    emit pitchChanged(pitch * rad2deg);
    emit rollChanged(roll * rad2deg);
    emit yawChanged(yaw * rad2deg);
    emit longitudeChanged(ahrsMsg.gps.longitude);
    emit latitudeChanged(ahrsMsg.gps.latitude);
}

void NavigationController::process() {
    ROS_INFO("Starting navigation controller thread");
    ros::Subscriber ahrsSubscriber = mNh.subscribe("/ahrs/ahrs_status", 1, &NavigationController::ahrs_ros_cb, this);

    ros::Rate r(10);
    while(mNh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    ROS_WARN("Navigation Converted controller exited");
}
