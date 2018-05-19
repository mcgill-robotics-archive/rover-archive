//
//Created by Vanessa Roe on 27/01/18
//vanessa.roe (at) mail.mcgill.ca
//

//called in: /hci_cpp/src/hci/src/controller/MainController.h
//           /hci_cpp/src/hci/src/controller/ScienceContoller.cpp

#ifndef HCI_CPP_SCIENCECONTROLLER_H
#define HCI_CPP_SCIENCECONTROLLER_H

#include <QtWidgets/QWidget>
#include <ros/ros.h>
#include "JoystickInterface.h"
#include <model/ScienceData.h>
#include <std_msgs/Float64.h>

class ScienceController : public JoystickInterface {
Q_OBJECT

public:
    ScienceController(ros::NodeHandle &nh);

    virtual ~ScienceController() {};

    virtual void handleJoystickData(JoystickData data);

public slots:
    void process();

signals:
    void motorEnableChanged(bool enable);
    void probeSpeedUpdate(float probe);
    void drillSpeedUpdate(float drill);
    void carriageSpeedUpdate(float carriage);

private:
    ros::NodeHandle& sNodeHandle;
    ros::Publisher sCommandPublisher;
    
    void publish();
    void enableMotors(bool enable);
    void probeSpeed(float probe);
    void drillSpeed(float drill);
    void carriageSpeed(float carriage);

    bool motorEnable; //wether the motors are enabled or not
    std_msgs::Float64 carriage; //speed of carriage (to be published)
    float currPSpeed; //current speed of probe
    float currDSpeed; //current speed of drill
    float currCSpeed; //current speed of carriage

};

#endif
