//
// Created by David Lavoie-Boutin on 25/07/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_NAVIGATIONCONTROLLER_H
#define HCI_CPP_NAVIGATIONCONTROLLER_H

#include <QObject>
#include <ros/ros.h>
#include <ahrs/AhrsStdMsg.h>
#include <tf/tf.h>


class NavigationController : public QObject {
Q_OBJECT
public:
    NavigationController(ros::NodeHandle &nh);

    virtual ~NavigationController() {};

public slots:
    void process();

signals:
    void pitchChanged(float value);
    void rollChanged(float value);
    void yawChanged(float value);
    void longitudeChanged(float value);
    void latitudeChanged(float value);

private:
    ros::NodeHandle mNh;
    double roll, pitch, yaw;
    tf::Quaternion quat;

    void ahrs_ros_cb(const ahrs::AhrsStdMsg& ahrsMsg);

};


#endif //HCI_CPP_NAVIGATIONCONTROLLER_H
