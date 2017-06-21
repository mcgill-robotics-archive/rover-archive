//
// Created by David Lavoie-Boutin on 19/06/17.
// david.lavoie-boutin (at) mail.mcgill.ca
//

#ifndef HCI_CPP_DRIVECONTROLLER_H
#define HCI_CPP_DRIVECONTROLLER_H

#include <QtWidgets/QWidget>
#include <ros/ros.h>
#include <model/DriveData.h>
#include "JoystickInterface.h"
#include <rover_common/MotorStatus.h>

/**
 * @brief Class responsible for publishing drive commands and subscribing
 * to drive related status information from the rover.
 *
 * Class also implements the joystick data parsing and builds the message
 * to be published to the ros environment.
 */
class DriveController : public JoystickInterface {
Q_OBJECT
public:

    /**
     * @brief Constructor
     * @param nh Reference to the node handle to use for publishing and subscribing
     */
    DriveController(ros::NodeHandle& nh);

    virtual ~DriveController() {};

    virtual void handleJoystickData(JoystickData& data);

public slots:
    /**
     * @brief Update the steering information for publisher
     *
     * @param mode New mode to publish with
     */
    void updateSteeringMode(SteeringMode mode);


    /**
     * @brief Thread start entry point
     *
     * See DCDCConverter class for more details
     */
    void process();

signals:
    void steeringModeUpdated(SteeringMode mode);
    void motorEnableChanged(bool enable);
    void wheelStatusUpdated(DriveStatusData status);

private:
    ros::NodeHandle& mNodeHandle;
    ros::Publisher mCommandPublisher;

    // Status data to be published
    SteeringMode mSteeringMode;
    bool mMotorEnabled;
    float mLinearVel;
    float mAngularVel;

private slots:
    void enableMotors(bool enable);
    void setVelocityCommand(float linear, float angular);
    void publish();
    void wheelStatusROSCallback(const rover_common::MotorStatus& message);
};


#endif //HCI_CPP_DRIVECONTROLLER_H
