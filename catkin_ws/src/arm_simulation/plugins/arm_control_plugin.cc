// Control plugin for Gazebo

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{
  class ArmControlPlugin : public ModelPlugin
  {
    public:
      void callBack(const std_msgs::Float64::ConstPtr &msg, const std::string &joint)
      {
        this->joints[joint].prevVel = this->joints[joint].velocity;
        this->joints[joint].velocity = msg->data;
      }

      // Called by gazebo when plugin is loaded.
      void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
      {
        this->model = _parent;
        physics::Joint_V jointVector = this->model->GetJoints();
         
        for(int i = 0; i < jointVector.size(); ++i)
        {
          std::string jointName = jointVector[i]->GetName();
          
          // Skip world_fix because it is fixed
          if(jointName == "world_fix") continue;
          
          std::string modelName = this->model->GetName();
          std::string subPath = "/" + modelName + "/" + jointName + "_velocity_controller/command";
      
          JOINT jointData;
          jointData.name = jointName;
          jointData.velocity = 0.0;
          jointData.prevVel = 0.0;
          jointData.jointPointer = this->model->GetJoint(jointName);
          jointData.brakesOn = false;
          jointData.timer = 0.0;
          jointData.jointStateSub = this->rosNode.subscribe<std_msgs::Float64>(subPath, 1, boost::bind(&ArmControlPlugin::callBack, this, _1, jointName));
            
          this->joints[jointName] = jointData;
        }

        // Listen to the update event. This event is broadcast every simulation iteration
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ArmControlPlugin::OnUpdate, this, _1));
        // Publish the state for Rviz, Moveit
        this->joint_state_pub = rosNode.advertise<sensor_msgs::JointState>("/arm/joint_states", 1);      
      }

      // Called by the world update end event
      void OnUpdate(const common::UpdateInfo & /*_info*/)
      {
        double now, newVel, prevVel;
        sensor_msgs::JointState jointStates;
        jointStates.header = std_msgs::Header();

        for(auto jointsEl=this->joints.begin(); jointsEl!=joints.end(); ++jointsEl)
        {
          std::string jointName = jointsEl->first;
          JOINT *jointData = &jointsEl->second;

          newVel = jointData->velocity;
          prevVel = jointData->prevVel;
          
          // If the brakes are not already on
          // We brake only if the joint was not moving before (prevVel == 0.0)
          // and if we start moving the joint (newVel != 0.0)
          if(!jointData->brakesOn && prevVel == 0.0 && newVel != 0.0) {            
            jointData->brakesOn = true;
            jointData->timer = ros::Time::now().toSec();
          }
          
          if(jointData->brakesOn) {
            now = ros::Time::now().toSec();
            if((now-jointData->timer) >= this->delay) {
              jointData->brakesOn = false;
            }
          }
           
          if(!jointData->brakesOn) {
            // First argument is the index of the axis of that particular joint.
            // Since it's a simple rotation, there's only one axis
            jointData->jointPointer->SetVelocity(0, newVel);
          } else {
            jointData->jointPointer->SetVelocity(0, 0);
          }

          jointStates.header.stamp = ros::Time::now();
          jointStates.name.push_back(jointName);
          jointStates.position.push_back(jointData->jointPointer->GetAngle(0).Radian());
          jointStates.velocity.push_back(jointData->jointPointer->GetVelocity(0));
        }

        // Send Joint states
        this->joint_state_pub.publish(jointStates);
      }

    private:
      physics::ModelPtr model;
      ros::NodeHandle rosNode;
      
      // Delay between next command and the time it move
      double delay = 0.2;

      struct JOINT
      {
        std::string name;
        double velocity;
        double prevVel;
        physics::JointPtr jointPointer;
        bool brakesOn;
        double timer;
        ros::Subscriber jointStateSub;
      };
      std::map<std::string, JOINT> joints;
      
      // ROS publisher
      ros::Publisher joint_state_pub;

      // Pointer to the update event connection, 'updateConnection'
      event::ConnectionPtr updateConnection;      
  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ArmControlPlugin)
}
