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
    else if (armMode == CLOSED){	//if the arm is in CLOSED mode
        if (data.buttons[2]) 			//if button #3 was pressed on joystick, change to position
            changeCloseLoopMode(POSITION);
        else if (data.buttons[3])		//if button #4 was pressed on joystick, change to orientation
            changeCloseLoopMode(ORIENTATION);
        if(motorEnable){			//update the position and orientation of arm ONLY if motors are enabled
             if (closedLoopMode == POSITION)		//if in POSITION, change the POINT values of message
                 changeArmPoint(data.a1, data.a2, data.a3);
             else if (closedLoopMode == ORIENTATION)	//if in ORIENTATION, change the QUATERNION values of message
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


/*Input: ArmMode mode (either 'OPEN' or 'CLOSED') 
  Output: no output, but update the private variable ArmMode 'armMode'
  Method: Write info in the termial saying the mode has been changed. If mode is the same as the 
    current value of armMode, do nothing. Otherwise, update armMode to the new value, emit the 
    signal ('armModeChange') with the new mode
*/
void ArmController::changeArmMode(ArmMode mode) {
    ROS_INFO("ArmController.cpp: Arm mode changed");
    if (armMode != mode)
    {
        armMode = mode;
        emit armModeChanged(mode);
    }

}


/*Input: ArmJoint joint (either 'BASE', 'D1', 'D2' or 'END_EFFECTOR') 
  Output: no output, but update the private variable ArmJoint 'activeJoint'
  Method: Write info in the termial saying the arm joint has been changed. If arm joint is the 
    same as the current value of activeJoint, do nothing. Otherwise, update activeJoint to the 
    new value, and emit the signal ('armJointChanged') with the new joint
*/
void ArmController::changeOpenLoopJoint(ArmJoint joint) {
    ROS_INFO("ArmController.cpp: Active joint changed");
    if (activeJoint != joint)
    {
        activeJoint = joint;
        emit armJointChanged(joint);
    }
}


/*Input: ArmClosedLoop mode (either 'POSITION' or 'ORIENTATION') 
  Output: no output, but update the private variable ArmClosedLoopMode 'closedLoopMode'
  Method: Write info in the termial saying the mode has been changed. If mode is the same as the 
    current value of closedLoopMode, do nothing. Otherwise, update closedLoopMode to the new mode 
    and emit the signal ('closedLoopModeChanged')with the new mode
*/ 
void ArmController::changeCloseLoopMode(ArmClosedLoopMode mode) {
    ROS_INFO("ArmController.cpp: ClosedLoop mode changed");
    if (closedLoopMode != mode){
        closedLoopMode = mode;
        emit closedLoopModeChanged(mode);
    }
}


/*Input: no input
  Output: no ouput, but publish the proper message
  Method: if the arm is in OPEN mode, check if the motors are enabled. If they are, publish the 
    'jointVelocities' message to mCommandPublisher PUBLISHER. If the motors aren't enabled, create 
    and publish an empty message ('jointVelocitiesEmpty').
    If the arm is in CLOSED mode, check if the motors are enabled. If they are, publish the POSE 
    message ('closeArm') to cCommandPublisher PUBLISHER. If the motors aren't enabled, publish an 
    emtpy POSE message ('emptyPose')
*/
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
	    //ROS_INFO("ArmController.cpp /n pos.x: /t%f /n pos.y:/t%f /n pos.z:/t%f /n ori.x:/t%f /n ori.y:/t%f /n ori.z:/t%f /n ori.w:/t%f", closeArm.position.x, closeArm.position.y, closeArm.position.z,closeArm.orientation.x, closeArm.orientation.y, closeArm.orientation.z, closeArm.orientation.w);
       }
       else{
            cCommandPublisher.publish(emptyPose);
       }
    }
}


/*Input: 1 boolean (enable) (signals whether or not the motors should be able to run)
  Output: no output, but change the private bool 'motorEnable' which is used elsewhere in the program
  Method: If enable is the same as the current value of motorEnable, do nothing. Otherwise, update
    motorEnable to the new value, emit the signal ('motorEnableChanged')saying it has been changed with
    the new value, and output information into the termial saying it has been changed and what it has 
    been changed to.  
*/
void ArmController::enableMotors(bool enable) {
    if (motorEnable != enable)
    {
        motorEnable = enable;
        emit motorEnableChanged(motorEnable);
        ROS_INFO("ArmController.cpp: Changed motor enable to %s", (motorEnable ? "true" : "false"));
    }
}


/*Input: 3 float values, (a1), (a2) and (a3). 
  Output: no ouput, but change the POINT part (called 'position') of the POSE message ('closeArm')
  Method: Update the x, y and z axises of closeArm.position directly
*/
void ArmController::changeArmPoint(float a1, float a2, float a3){
    closeArm.position.x = a1;
    closeArm.position.y = a2;
    closeArm.position.z = a3;
}


/*Input: 3 float values, representing yaw (a1), roll (a2) and pitch (a3). 
  Output: no ouput, but change the QUATERNION part (called 'orientation') of the POSE message ('closeArm')
  Method: take the three input and preform calculations on them to convert them into the proper x,y,z and w 
    axises need by the QUATERNION message. (For more information, look up 'rpy to quaternion') After these
    calculations, update the parts of closeArm.orientation
*/
void ArmController::changeArmQuad(float a1, float a2, float a3){
    a1 = yaw, a2 = roll, a3 = pitch
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



