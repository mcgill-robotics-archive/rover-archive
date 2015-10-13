//
// Created by David Lavoie-Boutin on 15-10-08.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ahrs/AhrsStdMsg.h>
#include <ahrs/CenterWorldFrame.h>
#include <tf/transform_broadcaster.h>

ahrs::AhrsStdMsg msg;
ahrs::AhrsStdMsg tmp_msg;

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

bool serviceCallback( 
        ahrs::CenterWorldFrame::Request & request, 
        ahrs::CenterWorldFrame::Response & response)
{
    static tf::TransformBroadcaster br;
    if (request.GO)
    {
        msg = tmp_msg;
        response.Done = true;
    }
    return true;
}

void poseCallback(const ahrs::AhrsStdMsg & ahrsMsg)
{
    tmp_msg = ahrsMsg;

    broadcast(
        tf::Quaternion(
            ahrsMsg.pose.pose.orientation.x,
            ahrsMsg.pose.pose.orientation.y,
            ahrsMsg.pose.pose.orientation.z,
            ahrsMsg.pose.pose.orientation.w),
        tf::Vector3(
            ahrsMsg.pose.pose.position.x,
            ahrsMsg.pose.pose.position.y,
            ahrsMsg.pose.pose.position.z),
        "ahrs");
    tf::Quaternion quaternion; 
    broadcast(
        quaternion,
        tf::Vector3(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z),
        "world");

}



int main(int argc, char ** argv)
{
    ros::init(argc, argv, "tf_broadcaster");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("ahrs_status", 10, &poseCallback);
    ros::ServiceServer service = nh.advertiseService("center_world", &serviceCallback); 
    

    ros::spin();
    return 0;
}

