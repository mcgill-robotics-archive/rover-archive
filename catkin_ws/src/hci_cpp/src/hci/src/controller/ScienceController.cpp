//
//Created by Vanessa Roe on 27/01/18
// vanessa.roe (at) mail.mcgill.ca
//

#include "ScienceController.h"

ScienceController::ScienceController(ros::NodeHandle &nh): sNodeHandle(nh) { 
     sCommandPublisher = sNodeHandle.advertise<std_msgs::Float64>("science_test", 100);
}

void ScienceController::handleJoystickData (JoystickData data) {
    enableMotors(data.buttons[0]);

    if (data.buttons[2]) {
        probeSpeed(60.0);
    }
    else if (data.buttons[3]) {
        probeSpeed(20.0); 
    }
    else {
        probeSpeed(0.0);
    }
  
    if(motorEnable){			//if the motors are enabled, 
        drillSpeed(data.a4 *-4000); 	//drill speed is axsis4 * -4,000
        carriage.data = data.a2 * -4000.0;//carriage speed based on axsis2 * -40000
        carriageSpeed(data.a2 * -40000.0);
    } 
    else {				//if motors aren't enabled, they won't run
        drillSpeed(0.0);
        carriage.data = 0.0;
        carriageSpeed(0.0);
    }

    //We need to find a way to include the hat data or we can change what we use to adjust the carraige 
	//if data.hat_top: carriage +=1;
	//if data.hat_down: carraige -=1;

}

void ScienceController::process(){
    ROS_INFO("Starting science controller thread");
    ros::Rate rate (10);

    while(sNodeHandle.ok()){
	publish();
        ros::spinOnce();
        rate.sleep();
    }
}

void ScienceController::publish(){
    if(motorEnable) {
        sCommandPublisher.publish(carriage);
    }
}

//determine wether the motors should be enabled or not, emit it in motorEnableChanged
void ScienceController::enableMotors(bool enable) {
    if(motorEnable != enable) {
         motorEnable = enable;
         emit motorEnableChanged(motorEnable);
         ROS_INFO("ScienceController.cpp: Changed motor enable to %s", motorEnable ? "true":"false");
    }
}

//update the probe speed to the given value, emit it in probeSpeedUpdate
void ScienceController::probeSpeed(float probe){
    if(currPSpeed != probe){
        currPSpeed = probe;
        emit probeSpeedUpdate(currPSpeed);
        ROS_INFO("ScienceController.cpp: Probe speed changed to %f", currPSpeed);
    }
}

//update the drill speed to given value, emit it in drillSpeedUpdate
void ScienceController::drillSpeed(float drill){
    if(currDSpeed != drill){
        currDSpeed = drill;
        emit drillSpeedUpdate(currDSpeed);
        ROS_INFO("ScienceController.cpp: Drill speed changed to %f", currDSpeed);
    }
}

//update the carriage speed to given value, emit to the hci display in carriageSpeedUpdate
void ScienceController::carriageSpeed(float carriage){
    if(currCSpeed != carriage){
        currCSpeed = carriage;
        emit carriageSpeedUpdate(currCSpeed);
        ROS_INFO("ScienceController.cpp: Carriage speed changed to %f", currCSpeed);
    }
}




