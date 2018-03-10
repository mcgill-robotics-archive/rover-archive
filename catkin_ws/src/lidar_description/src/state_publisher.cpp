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
    odom_trans.header.frame_id = "base";        // Probably the world
    odom_trans.child_frame_id = "base_link";    // Probably the lidar

    while(ros::ok())
    {
        // update joint state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(1);
        joint_state.position.resize(1);
        joint_state.name[0] = "tilt_lidar";
        joint_state.position[0] = tilt_lidar;

        // update transform -- Placeholder
        odom_trans.header.stamp = ros::Time::now();     // The time stamp needed for tf transforms/broadcasts
        odom_trans.transform.translation.x = 0;         // The x, y and z translations (changes/deltas) from base (as parent) to base_link (as child), i.e.
        odom_trans.transform.translation.y = 0;         // how far are/should we move from the parent (the world) as the child (the lidar).
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

        // send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // Here is where you would update your robot state.
        tilt_lidar = tilt_lidar + 0;

        // This will adjust as needed per iteration
        loop_rate.sleep(); // sleeps for however long is left until "loop_rate" (pads the loop to run in 30hz)
    }

    return 0;
}
