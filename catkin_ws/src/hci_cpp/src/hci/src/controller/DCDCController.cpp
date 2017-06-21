//
// Created by David Lavoie-Boutin on 21/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#include <ros/subscriber.h>
#include "DCDCController.h"

DCDCController::DCDCController(ros::NodeHandle &nh) : mNh(nh){ }

void DCDCController::process() {
    ROS_INFO("Starting dcdc controller thread");
    ros::Subscriber input_currentSubscriber = mNh.subscribe("/dcdc_nuc/input_current", 1, &DCDCController::input_current_ros_cb, this);
    ros::Subscriber input_voltageSubscriber = mNh.subscribe("/dcdc_nuc/input_voltage", 1, &DCDCController::input_voltage_ros_cb, this);
    ros::Subscriber output_currentSubscriber = mNh.subscribe("/dcdc_nuc/output_current", 1, &DCDCController::output_current_ros_cb, this);
    ros::Subscriber output_powerSubscriber = mNh.subscribe("/dcdc_nuc/output_power", 1, &DCDCController::output_power_ros_cb, this);
    ros::Subscriber output_voltageSubscriber = mNh.subscribe("/dcdc_nuc/output_voltage", 1, &DCDCController::output_voltage_ros_cb, this);
    ros::Subscriber temperatureSubscriber = mNh.subscribe("/dcdc_nuc/temperature", 1, &DCDCController::temperature_ros_cb, this);

    while(mNh.ok())
    {
        ros::spinOnce();
    }
    ROS_WARN("DCDC Converted controller exited");
}

void DCDCController::input_current_ros_cb(const std_msgs::Float64 &value) {
    emit InputCurrentUpdated(value.data);
}

void DCDCController::input_voltage_ros_cb(const std_msgs::Float64 &value) {
    emit InputVoltageUpdated(value.data);
}

void DCDCController::output_current_ros_cb(const std_msgs::Float64 &value) {
    emit OutputCurrentUpdated(value.data);
}

void DCDCController::output_power_ros_cb(const std_msgs::Float64 &value) {
    emit OutputPowerUpdated(value.data);
}

void DCDCController::output_voltage_ros_cb(const std_msgs::Float64 &value) {
    emit OutputVoltageUpdated(value.data);
}

void DCDCController::temperature_ros_cb(const std_msgs::Float64 &value) {
    emit TemperatureUpdated(value.data);
}
