#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h> // Daniel: Needed to create a broadcaster
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

const int PI = 3.1415926535897931;

namespace gazebo {   
    class TiltUnitPlugin : public ModelPlugin {
      public:
        void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
            ROS_INFO("tilting_unit_gazebo_plugin loaded!");

            this->model = parent;

            joint_states_topic= "/joint_states";
            
            x_velocity_topic = "/x_velocity_lidar";
            y_velocity_topic = "/y_velocity_lidar";
            z_velocity_topic = "/z_velocity_lidar";

            navigation_twist_topic = "/cmd_vel";

            tilting_unit_joint_state.name.resize(1);
            tilting_unit_joint_state.position.resize(1);
            tilting_unit_joint_state.velocity.resize(1);
            tilting_unit_joint_state.effort.resize(1);

            tilting_unit_joint_state.position[0] = 0.0;
            tilting_unit_joint_state.velocity[0] = 0.0;
            tilting_unit_joint_state.effort  [0] = 0.0;

            tilting_unit_joint_state.name[0] = "tilt_lidar";

            tilting_unit_joint = model->GetJoint(tilting_unit_joint_state.name[0]);

            if (tilting_unit_joint == NULL) {
                ROS_FATAL("The controller couldn't get joint %s", tilting_unit_joint_state.name[0].c_str());
            }

            theta_max = PI/3;
            theta_min = -PI/3;
            period = 1.0;
            rot_speed = 2*(theta_max - theta_min)/period;

            tilting_unit_joint->SetPosition(0, 0);

            joint_state_pub = node.advertise<sensor_msgs::JointState>(joint_states_topic, 1);

            x_velocity_sub = node.subscribe<std_msgs::Float32>(x_velocity_topic, 10, boost::bind(&TiltUnitPlugin::x_velocity_callback, this, _1));
            y_velocity_sub = node.subscribe<std_msgs::Float32>(y_velocity_topic, 10, boost::bind(&TiltUnitPlugin::y_velocity_callback, this, _1));
            z_velocity_sub = node.subscribe<std_msgs::Float32>(z_velocity_topic, 10, boost::bind(&TiltUnitPlugin::z_velocity_callback, this, _1));

            navigation_twist_subscriber = node.subscribe<geometry_msgs::Twist>(navigation_twist_topic, 10, boost::bind(&TiltUnitPlugin::navigation_twist_callback, this, _1));

            update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TiltUnitPlugin::OnUpdate, this));

            // Daniel's:
            tf::TransformBroadcaster broadcaster;
            this->init = false; // Set to true, once model's Pose.z > 0.5
        }

        // Called by the world update start event
        void OnUpdate() {
            // Daniel's (START)

            // Instantiate a ModelState object to use in Gazebo library call to get current pose (returns a math_pose 
            gazebo::physics::ModelState model_state = gazebo::physics::ModelState(this->model);
            gazebo::math::Pose math_pose = model_state.GetPose();

            // I.e. If we haven't yet reached 0.5, set z.velocity to 0.5 and continue simulating (nb it wouldn't simulate if stuck in a while loop)
            if (!this->init) {
                this->model->SetLinearVel(math::Vector3(0, 0, 0.5));
                if ( math_pose.pos.z > 0.5 ) {
                    this->model->SetLinearVel(math::Vector3(0, 0, 0));
                    this->init = true;
                }
            }

            // Fully construct a geometry Pose (instead of a math one)
            geometry_msgs::Pose cur_pose;

            cur_pose.position.x = math_pose.pos.x;
            cur_pose.position.y = math_pose.pos.y;
            cur_pose.position.z = math_pose.pos.z;

            cur_pose.orientation.x = math_pose.rot.x;
            cur_pose.orientation.y = math_pose.rot.y;
            cur_pose.orientation.z = math_pose.rot.z;
            cur_pose.orientation.w = math_pose.rot.w;


            // TF stamped msg, composed of: 'std_msgs/Header' header, string child_frame_id and a geometry_msgs/Transform 
            geometry_msgs::TransformStamped trans_msg;
            trans_msg.header.frame_id = "base";         // Probably the world
            trans_msg.header.stamp = ros::Time::now();
            trans_msg.child_frame_id = "base_link";     // Probably the lidar('s base)

            // 'Transform transform' attribute == "the transform between two coordinate frames in free space" using a Vector3 and a Quaternion (both geometry_msg)
            trans_msg.transform.translation.x = cur_pose.position.x;
            trans_msg.transform.translation.y = cur_pose.position.y;
            trans_msg.transform.translation.z = cur_pose.position.z;
            trans_msg.transform.rotation = cur_pose.orientation;


            // Updates by broadcasting the current transform/translation
            broadcaster.sendTransform(trans_msg);

            // (END)

            if(theta_max == theta_min) {
              tilting_unit_joint->SetPosition(0, 0);
              return;
            }

            double joint_angle = tilting_unit_joint->GetAngle(0).Radian();

            if( (joint_angle >= theta_max && rot_speed > 0.0) || (joint_angle <= theta_min && rot_speed < 0.0) ) {
                rot_speed *= -1;
            }

            // Daniel: Only if we're finished getting the lidar high enough, do we want to modify the linear velocity with our topics
            if(this->init) {
                this->model->SetLinearVel(math::Vector3(x_velocity, y_velocity, z_velocity));
                this->model->SetAngularVel(math::Vector3(x_rotation, y_rotation, z_rotation));
                
                //this->model->SetWorldTwist(math::Vector3(x_velocity, y_velocity, z_velocity), math::Vector3(x_rotation, y_rotation, z_rotation), true);
            }
          
            tilting_unit_joint->SetEffortLimit(0, 5.0);
            tilting_unit_joint->SetVelocity(0, rot_speed);

            tilting_unit_joint_state.header.stamp = ros::Time::now();

            tilting_unit_joint_state.position[0] = tilting_unit_joint->GetAngle(0).Radian();
            tilting_unit_joint_state.velocity[0] = tilting_unit_joint->GetVelocity(0);

            joint_state_pub.publish(tilting_unit_joint_state);
            
            

            return;
        }

        void x_velocity_callback(const std_msgs::Float32::ConstPtr &velocity)
        {
            this->x_velocity = velocity->data;
        }

        void y_velocity_callback(const std_msgs::Float32::ConstPtr &velocity)
        {
            this->y_velocity = velocity->data;
        }

        void z_velocity_callback(const std_msgs::Float32::ConstPtr &velocity)
        {
            this->z_velocity = velocity->data;
        }

        void navigation_twist_callback(const geometry_msgs::Twist::ConstPtr &twist)
        {
            geometry_msgs::Vector3 linear = twist->linear;
            geometry_msgs::Vector3 angular = twist->angular;
            this->x_velocity = linear.x;
            this->y_velocity = linear.y;
            this->z_velocity = linear.z;
            this->x_rotation = angular.x;
            this->y_rotation = angular.y;
            this->z_rotation = angular.z;
        }

      private:
        event::ConnectionPtr update_connection; // Pointer to the update event connection
        ros::NodeHandle node;                   // ROS Nodehandle
        ros::Publisher joint_state_pub;         // ROS Subscribers and Publishers
        ros::Subscriber x_velocity_sub;
        ros::Subscriber y_velocity_sub;
        ros::Subscriber z_velocity_sub;
        ros::Subscriber navigation_twist_subscriber;

        std::string joint_states_topic;
        std::string x_velocity_topic;
        std::string y_velocity_topic;
        std::string z_velocity_topic;
        std::string navigation_twist_topic;

        double theta_max; //[rad]
        double theta_min; //[rad]
        double period;    //[sec]
        double rot_speed; //[rad/sec]

        float x_velocity;
        float y_velocity;
        float z_velocity;

        float x_rotation;
        float y_rotation;
        float z_rotation;
        
        physics::ModelPtr model;
        physics::JointPtr tilting_unit_joint;
        sensor_msgs::JointState tilting_unit_joint_state;
      
        tf::TransformBroadcaster broadcaster; // Daniel: TF broadcaster
        bool init; // Daniel: used to levitate the lidar

    }; // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(TiltUnitPlugin)

}
