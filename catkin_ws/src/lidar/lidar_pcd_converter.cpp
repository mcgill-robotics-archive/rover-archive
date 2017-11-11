//
// Created by Michael Vaquier on 11/11/17
//

#include "ros/ros.h"
#include <lidar/LidarStdMsg.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>


void mySigintHandler(int sig)
{
    ROS_INFO("ahrs_node terminated");
    ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_pcd_converter");
  ros::NodeHandle nh;
  signal(SIGINT, mySigintHandler);

  ros::Subscriber sub = nh.subscribe("/scan", 1000, scanCallback);
  ros::Subscriber subStabilizer = nh.subscribe("/lidar/lidar_angle", 1000, stabilizerCallback);
  ros::Publisher lidarPublisher = nh.advertise<sensor_msgs::PointCloud>("/lidar/lidar_point_cloud", 100);

  ros::spin();

  return 0;
}

laser_geometry::LaserProjection projector_;
tf::TransformListener listener_;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/base_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
  }

  sensor_msgs::PointCloud cloud;
  projector_.transformLaserScanToPointCloud("/base_link",*scan_in,
          cloud,listener_);

  lidarPublisher.publish(cloud);
}

void stabilizerCallback (const geometry_msgs::Vector3::ConstPtr& rollPitchYaw) {
    tf::Quaternion q_tf;
    q_tf.setRPY(rollPitchYaw.x,rollPitchYaw.y,rollPitchYaw.z);
    lidarStabilizer.publish(q_tf);
    return;
}