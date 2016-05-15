//
// Created by David Lavoie-Boutin on 2016-01-31.
//

#ifndef ROVER_ARDUINO_TRANSFORMSENDER_H
#define ROVER_ARDUINO_TRANSFORMSENDER_H

#include <ros.h>
#include <tf/transform_broadcaster.h>
#include "TransformConfig.h"
#include "arm_control/JointPosition.h"

namespace arm {


/**
 * \brief This class wants to wrap the mechanics of sending transforms for
 * the rover 6 DOF arm.
 */
class TransformSender {
public:
    /**
     * \brief Constructor configures the internal transforms with the proper
     * name and static distances from the configuration object.
     *
     * After the object is instantiated, the <code> init() </code> method
     * should be called to fully register the sender.
     *
     * \param nh Pointer to a valid node handle for logging and publishing
     * transforms.
     * \param config A configured TransfromConfig object containing the
     * names and distances for all the transforms
     */
    TransformSender(ros::NodeHandle *nh, TransformConfig &config);

    /**
     * \brief Proceed to correctly initialise the transform sender.
     *
     * This method registers the tfBroadcaster with the ros master and sends
     * a first all zero set of transforms.
     */
    void init(ros::Publisher *publisher);

    /**
     * \brief Update the current angle of each transform.
     *
     * Note all angles are in radians
     */
    void updateRotations(float yaw_base, float pitch_base, float pitch2, float roll1, float pitch3, float roll2);

    /**
     * \brief Updates the timestamps and sends the transform.
     */
    void sendTransforms();
    virtual ~TransformSender();

    /**
     * \brief Convert a RPY angle to a quaternion
     *
     * \param roll Roll angle in radians
     * \param pitch Pitch angle in radians
     * \param yaw Yaw angle in radians
     * \param quaternion A quaternion object which will be updated with the
     * newly computed quaternion parameters
     */
    void from_euler(float roll, float pitch, float yaw, geometry_msgs::Quaternion & quaternion);

private:
    geometry_msgs::TransformStamped baseYaw;
    geometry_msgs::TransformStamped mPitch1;
    geometry_msgs::TransformStamped mPitch2;
    geometry_msgs::TransformStamped mRoll1;
    geometry_msgs::TransformStamped mPitch3;
    geometry_msgs::TransformStamped mRoll2;

    tf::TransformBroadcaster broadcaster;
    arm_control::JointPosition jointPosition;
    ros::Publisher * mPublisher;
    ros::NodeHandle * mNh;
    ros::Time mTime;
};
}

#endif //ROVER_ARDUINO_TRANSFORMSENDER_H
