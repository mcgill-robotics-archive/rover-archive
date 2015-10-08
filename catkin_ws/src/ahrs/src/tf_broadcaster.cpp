//
// Created by David Lavoie-Boutin on 15-10-08.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ahrs/AhrsStdMsg.h>
#include <tf/transform_broadcaster.h>


void broadcast(
        const tf::Quaternion & quaternion, 
        const tf::Vector3 & vector,
        const char* frame)
{
    static tf::TransformBroadcaster br;
    tf::Transform trf;

    trf.setOrigin(vector);
    trf.setRotation(quaternion);
    br.sendTransform(tf::StampedTransform(trf, ros::Time::now(), "map", frame));
}

void poseCallback(const ahrs::AhrsStdMsg & ahrsMsg)
{
    broadcast(
        tf::Quaternion(
            ahrsMsg.pose.pose.orientation.x,
            ahrsMsg.pose.pose.orientation.y,
            ahrsMsg.pose.pose.orientation.z,
            ahrsMsg.pose.pose.orientation.w),
        tf::Vector3(
            ahrsMsg.pose.pose.position.x/10.0,
            ahrsMsg.pose.pose.position.y/10.0,
            ahrsMsg.pose.pose.position.z/10.0),
        "ahrs");
}



int main(int argc, char ** argv)
{
    ros::init(argc, argv, "tf_broadcaster");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("ahrs_status", 10, &poseCallback);

    ros::spin();
    return 0;
}

