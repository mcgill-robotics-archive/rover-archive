#include "ros/ros.h"
#include <laser_assembler/AssembleScans.h>
#include <sensor_msgs/PointCloud.h>
using namespace laser_assembler;

int main(int argc, char **argv)
{
  ROS_INFO("Scan assembler node started");
  ros::init(argc, argv, "test_client");
  ros::NodeHandle n;
  ros::service::waitForService("assemble_scans");
  ros::ServiceClient client = n.serviceClient<AssembleScans>("assemble_scans");

  ros::Publisher pointCloudPublisher = n.advertise<sensor_msgs::PointCloud>("lidar_point_cloud", 100);

  while(ros::ok()) {
    AssembleScans srv;
    
    srv.request.begin = ros::Time::now() - ros::Duration(5);
    srv.request.end   = ros::Time::now();


    if (client.call(srv)) {
      ROS_INFO("Got cloud with %lu points\n", srv.response.cloud.points.size());
      pointCloudPublisher.publish(srv.response.cloud);
    } else {
      ROS_INFO("Service call failed\n");
    }
  }
  return 0;
}