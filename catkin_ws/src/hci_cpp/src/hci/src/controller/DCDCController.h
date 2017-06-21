//
// Created by David Lavoie-Boutin on 21/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_DCDCCONTROLLER_H
#define HCI_CPP_DCDCCONTROLLER_H

#include <QtWidgets/QWidget>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>


class DCDCController : public QObject {
Q_OBJECT
public:
    DCDCController(ros::NodeHandle &nh);
    virtual ~DCDCController() {};

public slots:
    void process();

signals:
    void InputVoltageUpdated(double);
    void InputCurrentUpdated(double);
    void OutputVoltageUpdated(double);
    void OutputCurrentUpdated(double);
    void OutputPowerUpdated(double);
    void TemperatureUpdated(double);

private:
    ros::NodeHandle mNh;


    void input_current_ros_cb(const std_msgs::Float64& value);
    void input_voltage_ros_cb(const std_msgs::Float64& value);
    void output_current_ros_cb(const std_msgs::Float64& value);
    void output_power_ros_cb(const std_msgs::Float64& value);
    void output_voltage_ros_cb(const std_msgs::Float64& value);
    void temperature_ros_cb(const std_msgs::Float64& value);

};


#endif //HCI_CPP_DCDCCONTROLLER_H
