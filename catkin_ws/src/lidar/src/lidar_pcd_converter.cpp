#include "ros/ros.h"
#include <lidar/LidarStdMsg.h>


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

  ros::Subscriber sub = n.subscribe("lidar_laser_scan", 1000, scanCallback);

  ros::Publisher ahrsPublisher = nh.advertise<lidar::LidarStdMsg>("lidar_point_cloud", 100);

  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_; 

  ros::spin();

  return 0;
}

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

  // Do something with cloud.
}