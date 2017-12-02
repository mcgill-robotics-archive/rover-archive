#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double tilt_lidar = 0;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "base";
    odom_trans.child_frame_id = "base_link";

    while(ros::ok())
    {
        // update joint state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(1);
        joint_state.position.resize(1);
        joint_state.name[0] = "tilt_lidar";
        joint_state.position[0] = tilt_lidar;

        // update transform -- Placeholder
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = 0;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

        // send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // Here is where you would update your robot state.
        tilt_lidar = tilt_lidar + 0;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}
