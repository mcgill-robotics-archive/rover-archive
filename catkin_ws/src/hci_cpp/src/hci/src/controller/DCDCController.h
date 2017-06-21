//
// Created by David Lavoie-Boutin on 21/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_DCDCCONTROLLER_H
#define HCI_CPP_DCDCCONTROLLER_H

#include <QtWidgets/QWidget>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>

/**
 * @brief Subscribe to the status information from the DCDC Converter and
 * emit signals with new data
 */
class DCDCController : public QObject {
    Q_OBJECT
public:
    DCDCController(ros::NodeHandle &nh);
    virtual ~DCDCController() {};

public slots:
    /**
     * @brief Start the subscribers and the ros spin loop.
     *
     * This should not be called directly, rather the object should be moved
     * to a dedicated thread and the thread's "started" signal connected to
     * this slot:
     *
     * @code
     * DCDCController controller(nh);
     * QThread *controllerThread = new QThread;
     * controller.moveToThread(controllerThread);
     * connect(controllerThread, &QThread::started, &controller, &DCDCController::process);
     * controllerThread->start();
     * @endcode
     */
    void process();

signals:
    /// Emited when subscriber receives new data point
    void InputVoltageUpdated(double);
    /// Emited when subscriber receives new data point
    void InputCurrentUpdated(double);
    /// Emited when subscriber receives new data point
    void OutputVoltageUpdated(double);
    /// Emited when subscriber receives new data point
    void OutputCurrentUpdated(double);
    /// Emited when subscriber receives new data point
    void OutputPowerUpdated(double);
    /// Emited when subscriber receives new data point
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
