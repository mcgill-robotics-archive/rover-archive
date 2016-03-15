//
// Created by David Lavoie-Boutin on 2016-01-18.
//

#ifndef ROVER_ARDUINO_MOTORCONTROLLER_H
#define ROVER_ARDUINO_MOTORCONTROLLER_H

#include <ros.h>
#include "MotorConfig.h"
namespace motor {

/**
 * \brief Motor Controller abstract interface.
 * 
 * Implements the factory design pattern. External classes should only be 
 * exposed to this interface. Calling the factory function with the proper 
 * configuration will instantiate the actual motor controller but the user
 * should only ever use the methods exposed in this interface.
 * 
 * The factory will return a pointer to the motor controller created. The caller 
 * is responsible for managing this pointer properly. 
 *
 * <code>
 *
 *     //Example calling syntax:
 *     int main () {
 *         drive::MotorConfig mMotorConfig;         
 *         mMotorConfig.ControllerType = drive::_MAXON;
 *         drive::MotorController mMotorController = MotorController::createMotorController(mMotorConfig);
 *         ...
 *         delete mMotorController;
 *         return 0;
 *     }
 * </code>
 */
class MotorController {
public:

    /**
     * \brief Public point of entry to create new motor controllers
     *
     * Checks the controller type in the configuration object to create the
     * correct controller type.
     *
     * \param motorConfig Configuration object containing the proper pin and type information
     * \param nodeHandle Pointer to a node handle for logging
     * \return Pointer to a new controller object which the user is responsible for managing.
     * Note that the pointer returned is generic type. Use caution if static casting to a sub-class.
     */
    static MotorController *createMotorController(MotorConfig motorConfig, ros::NodeHandle *nodeHandle);

    /**
     * \brief Interface method, sets the angular velocity of the wheel
     *
     * \param speed The desired speed output of the motor. The implementation
     * should account for setting the direction according to the sign.
     */
    virtual void setSpeed(int speed) = 0;

    /**
     * \brief Manually set the direction of the motor.
     *
     * <b> Note: </b> Will be overridden the next time <code>setSpeed()</code> is called.
     *
     * \param speed Signed speed value. The sign will drive the direction.
     */
    virtual void setDirection(int speed) = 0;

    /**
     * \brief Enable or disable the brakes on the motor.
     *
     * \param brk Boolean value true if the wheel should stop.
     */
    virtual void brake(bool brk) = 0;

    /**
     * \brief Enable or disable the motor controller
     *
     * \param en True value enables the motor controller.
     */
    virtual void enable(bool en) = 0;

    /**
     * \brief Obtain the current status of the motor controller feedback pins.
     *
     * This should be implemented using feedback pins from the motor controller
     *
     * \return True indicates the motor is operating normally.
     */
    virtual bool getStatus() = 0;


    virtual ~MotorController();

protected:
    MotorController();


};
}


#endif //ROVER_ARDUINO_MOTORCONTROLLER_H
