//
// Created by David Lavoie-Boutin on 23/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include "ArmController.h"

ArmController::ArmController(ros::NodeHandle &nh): nodeHandle(nh) {
    mCommandPublisher = nodeHandle.advertise<arm_control::JointVelocities>("joint_velocitiy", 100);
    cCommandPublisher = nodeHandle.advertise<geometry_msgs::Pose>("closed_arm", 100);
}

void ArmController::handleJoystickData(JoystickData data) {
    enableMotors(data.buttons[0]);

    if (data.buttons[6])
        changeArmMode(OPEN);
    else if (data.buttons[7])
        changeArmMode(CLOSED);

    if (armMode == OPEN)
    {
        if (data.buttons[8])
            changeOpenLoopJoint(BASE);
        else if (data.buttons[9])
            changeOpenLoopJoint(D1);
        else if (data.buttons[10])
            changeOpenLoopJoint(D2);
        else if (data.buttons[11])
            changeOpenLoopJoint(END_EFFECTOR);
        jointVelocities = arm_control::JointVelocities();
        if (activeJoint == BASE) {
            jointVelocities.base_yaw = data.a1;
            jointVelocities.base_pitch = data.a2;
        } else if (activeJoint == D1) {
            jointVelocities.diff_1_pitch = data.a2;
            jointVelocities.diff_1_roll = data.a1;
        } else if (activeJoint == D2) {
            jointVelocities.diff_2_pitch = data.a2;
            jointVelocities.diff_2_roll = data.a1;
        } else if (activeJoint == END_EFFECTOR){
            jointVelocities.end_effector = data.a2;
        }
    }
    else if (armMode == CLOSED){		//if the arm is in CLOSED mode
        if (data.buttons[2]) 			//if button #3 was pressed on joystick, change to position
            changeCloseLoopMode(POSITION);
        else if (data.buttons[3])		//if button #4 was pressed on joystick, change to orientation
            changeCloseLoopMode(ORIENTATION);
        if(data.buttons[1]){
             if (closedLoopMode == POSITION)	//if in POSITION,
                 changeArmPoint(data.a1, data.a2, data.a3);
             else if (closedLoopMode == ORIENTATION)//if in ORIENTATION
                 changeArmQuad(data.a1, data.a2, data.a3);
        }
    }
}

void ArmController::process() {
    ROS_INFO("Starting arm controller thread");
    ros::Rate rate(10);

    while (nodeHandle.ok())
    {
        publish();
        ros::spinOnce();
        rate.sleep();
    }
}

void ArmController::changeArmMode(ArmMode mode) {
    ROS_INFO("ArmController.cpp: Arm mode changed");
    if (armMode != mode)
    {
        armMode = mode;
        emit armModeChanged(mode);
    }

}

void ArmController::changeOpenLoopJoint(ArmJoint joint) {
    ROS_INFO("ArmController.cpp: Active joint changed");
    if (activeJoint != joint)
    {
        activeJoint = joint;
        emit armJointChanged(joint);
    }
}

void ArmController::changeCloseLoopMode(ArmClosedLoopMode mode) {
    ROS_INFO("ArmController.cpp: ClosedLoop mode changed");
    if (closedLoopMode != mode){
        closedLoopMode = mode;
        emit closedLoopModeChanged(mode);
    }
}

void ArmController::publish() {
    if (armMode == OPEN)
    {
        if (motorEnable)
            mCommandPublisher.publish(jointVelocities);
        else
        {
            arm_control::JointVelocities jointVelocitiesEmpty;
            mCommandPublisher.publish(jointVelocitiesEmpty);
        }
    }
    else if (armMode == CLOSED)
    {
        if (motorEnable){
            cCommandPublisher.publish(closeArm);
	    ROS_INFO("ArmController.cpp /n pos.x: /t%f /n pos.y:/t%f /n pos.z:/t%f /n ori.x:/t%f /n ori.y:/t%f /n ori.z:/t%f /n ori.w:/t%f", closeArm.position.x, closeArm.position.y, closeArm.position.z,closeArm.orientation.x, closeArm.orientation.y, closeArm.orientation.z, closeArm.orientation.w);
       }
       else{
            cCommandPublisher.publish(emptyPose);
       }
    }
}

void ArmController::enableMotors(bool enable) {
    if (motorEnable != enable)
    {
        motorEnable = enable;
        emit motorEnableChanged(motorEnable);
        ROS_INFO("ArmController.cpp: Changed motor enable to %s", (motorEnable ? "true" : "false"));
    }
}

void ArmController::changeArmPoint(float a1, float a2, float a3){
    closeArm.position.x = a1;
    closeArm.position.y = a2;
    closeArm.position.z = a3;
}

void ArmController::changeArmQuad(float a1, float a2, float a3){
    //a1 = yaw, a2 = roll, a3 = pitch
    double cy = cos(a1 * 0.5); 
    double sy = sin(a1 * 0.5);
    double cr = cos(a2 * 0.5); 
    double sr = sin(a2 * 0.5);
    double cp = cos(a3 * 0.5); 
    double sp = sin(a3 * 0.5);


    closeArm.orientation.w = (cy*cr*cp + sy*sr*sp);
    closeArm.orientation.x = (cy*sr*cp - sy*cr*sp);
    closeArm.orientation.y = (cy*cr*sp + sy*sr*cp);
    closeArm.orientation.z = (sy*cr*cp - cy*sr*sp);
}



