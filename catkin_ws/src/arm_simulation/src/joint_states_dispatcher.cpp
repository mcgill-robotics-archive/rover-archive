/*
 * Dispatches all the joints from the topic "/arm/joint_states" to individual topics
 * "/arm/[joint_name]_position_controller/state" for the pid_controller
 */

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"


ros::Publisher _base_pitch_publisher;
ros::Publisher _base_yaw_publisher;
ros::Publisher _elbow_pitch_publisher;
ros::Publisher _elbow_roll_publisher;
ros::Publisher _wrist_pitch_publisher;
ros::Publisher _wrist_roll_publisher;

void _dispatch_cb(const sensor_msgs::JointState::ConstPtr& msg) {

    std_msgs::Float64 bp_msg;
    bp_msg.data = msg->position[0];
    _base_pitch_publisher.publish(bp_msg);

    std_msgs::Float64 by_msg;
    by_msg.data = msg->position[1];
    _base_yaw_publisher.publish(by_msg);

    std_msgs::Float64 ep_msg;
    ep_msg.data = msg->position[2];
    _elbow_pitch_publisher.publish(ep_msg);

    std_msgs::Float64 er_msg;
    er_msg.data = msg->position[3];
    _elbow_roll_publisher.publish(er_msg);

    std_msgs::Float64 wp_msg;
    wp_msg.data = msg->position[4];
    _wrist_pitch_publisher.publish(wp_msg);

    std_msgs::Float64 wr_msg;
    wr_msg.data = msg->position[5];
    _wrist_roll_publisher.publish(wr_msg);
}

int main(int argc, char **argv) {

    // ROS init
    ros::init(argc, argv, "joint_states_dispatcher");
    ros::NodeHandle nh;

    ros::Subscriber _joint_states_subscriber = nh.subscribe("joint_states", 1, _dispatch_cb);

    _base_pitch_publisher = nh.advertise<std_msgs::Float64>("base_pitch_position_controller/state", 1);

    _base_yaw_publisher = nh.advertise<std_msgs::Float64>("base_yaw_position_controller/state", 1);

    _elbow_pitch_publisher = nh.advertise<std_msgs::Float64>("elbow_pitch_position_controller/state", 1);

    _elbow_roll_publisher = nh.advertise<std_msgs::Float64>("elbow_roll_position_controller/state", 1);

    _wrist_pitch_publisher = nh.advertise<std_msgs::Float64>("wrist_pitch_position_controller/state", 1);

    _wrist_roll_publisher = nh.advertise<std_msgs::Float64>("wrist_roll_position_controller/state", 1);

    ros::spin();

    return 0;
}

