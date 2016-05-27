//
// Created by David Lavoie-Boutin on 2016-01-31.
//

#include "TransformSender.h"
using namespace arm;

TransformSender::TransformSender(ros::NodeHandle *nh, TransformConfig &config) {
    mNh = nh;

    baseYaw.child_frame_id = config.baseYawFrame;
    mPitch1.child_frame_id = config.pitch1Frame;
    mPitch2.child_frame_id = config.pitch2Frame;
    mRoll1.child_frame_id = config.roll1Frame;
    mPitch3.child_frame_id = config.pitch3Frame;
    mRoll2.child_frame_id = config.roll2Frame;

    baseYaw.header.frame_id = config.baseFrame;
    mPitch1.header.frame_id = baseYaw.child_frame_id;
    mPitch2.header.frame_id = mPitch1.child_frame_id;
    mRoll1.header.frame_id = mPitch2.child_frame_id;
    mPitch3.header.frame_id = mRoll1.child_frame_id;
    mRoll2.header.frame_id = mPitch3.child_frame_id;

    baseYaw.transform.translation.x = config.armPosition;
    mPitch1.transform.translation.z = config.pitch1offset;
    mPitch2.transform.translation.x = config.pitch2offset;
    mRoll1.transform.translation.x = config.roll1offset;
    mPitch3.transform.translation.x = config.pitch3offset;
    mRoll2.transform.translation.x = config.roll2offset;
}

TransformSender::~TransformSender() {

}

void TransformSender::from_euler(float roll, float pitch, float yaw, geometry_msgs::Quaternion & quaternion)
{
    float cr2 = cos(roll*0.5);
    float cp2 = cos(pitch*0.5);
    float cy2 = cos(yaw*0.5);
    float sr2 = sin(roll*0.5);
    float sp2 = sin(pitch*0.5);
    float sy2 = sin(yaw*0.5);

    quaternion.w = cr2*cp2*cy2 + sr2*sp2*sy2;
    quaternion.x = sr2*cp2*cy2 - cr2*sp2*sy2;
    quaternion.y = cr2*sp2*cy2 + sr2*cp2*sy2;
    quaternion.z = cr2*cp2*sy2 - sr2*sp2*cy2;
}

void TransformSender::updateRotations(float yaw_base, float pitch_base, float pitch2, float roll1, float pitch3,
                                      float roll2) {
    from_euler(0, 0, yaw_base, baseYaw.transform.rotation);
    from_euler(0, pitch_base, 0, mPitch1.transform.rotation);
    from_euler(0, pitch2, 0, mPitch2.transform.rotation);
    from_euler(roll1, 0, 0, mRoll1.transform.rotation);
    from_euler(0, pitch3, 0, mPitch3.transform.rotation);
    from_euler(roll2, 0, 0, mRoll2.transform.rotation);

    jointPosition.base_pitch = yaw_base;
    jointPosition.base_yaw = pitch_base;
    jointPosition.diff_1_pitch = pitch2;
    jointPosition.diff_1_roll = roll1;
    jointPosition.diff_2_pitch = pitch3;
    jointPosition.diff_2_roll = roll2;
}

void TransformSender::sendTransforms() {
    mTime = mNh->now();
    baseYaw.header.stamp = mTime;
    mPitch1.header.stamp =mTime;
    mPitch2.header.stamp = mTime;
    mPitch3.header.stamp =mTime;
    mRoll1.header.stamp = mTime;
    mRoll2.header.stamp = mTime;

    broadcaster.sendTransform(baseYaw);
    broadcaster.sendTransform(mPitch1);
    broadcaster.sendTransform(mPitch2);
    broadcaster.sendTransform(mPitch3);
    broadcaster.sendTransform(mRoll1);
    broadcaster.sendTransform(mRoll2);

    mPublisher->publish(&jointPosition);
}

void TransformSender::init(ros::Publisher *publisher) {

    broadcaster.init(*mNh);
    updateRotations(0, 0, 0, 0, 0, 0);
    mPublisher = publisher;
    sendTransforms();
}
