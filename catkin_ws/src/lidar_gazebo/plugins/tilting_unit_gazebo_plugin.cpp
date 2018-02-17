#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

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

            update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TiltUnitPlugin::OnUpdate, this));
        }

        // Called by the world update start event
        void OnUpdate() {

            if(theta_max == theta_min) {
              tilting_unit_joint->SetPosition(0, 0);
              return;
            }

            double joint_angle = tilting_unit_joint->GetAngle(0).Radian();

            if( (joint_angle >= theta_max && rot_speed > 0.0) || (joint_angle <= theta_min && rot_speed < 0.0) ) {
                rot_speed *= -1;
            }
          
            tilting_unit_joint->SetEffortLimit(0, 5.0);
            tilting_unit_joint->SetVelocity(0, rot_speed);

            tilting_unit_joint_state.header.stamp = ros::Time::now();

            tilting_unit_joint_state.position[0] = tilting_unit_joint->GetAngle(0).Radian();
            tilting_unit_joint_state.velocity[0] = tilting_unit_joint->GetVelocity(0);

            joint_state_pub.publish(tilting_unit_joint_state);

            this->model->SetLinearVel(math::Vector3(x_velocity, y_velocity, z_velocity));

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

      private:
        event::ConnectionPtr update_connection; // Pointer to the update event connection
        ros::NodeHandle node;                   // ROS Nodehandle
        ros::Publisher joint_state_pub;         // ROS Subscribers and Publishers
        ros::Subscriber x_velocity_sub;
        ros::Subscriber y_velocity_sub;
        ros::Subscriber z_velocity_sub;

        std::string joint_states_topic;
        std::string x_velocity_topic;
        std::string y_velocity_topic;
        std::string z_velocity_topic;

        double theta_max; //[rad]
        double theta_min; //[rad]
        double period;    //[sec]
        double rot_speed; //[rad/sec]

        float x_velocity;
        float y_velocity;
        float z_velocity;
        
        physics::ModelPtr model;
        physics::JointPtr tilting_unit_joint;
        sensor_msgs::JointState tilting_unit_joint_state;

    }; // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(TiltUnitPlugin)

}
