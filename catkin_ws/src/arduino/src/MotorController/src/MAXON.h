//
// Created by David Lavoie-Boutin on 2016-01-15.
//

#ifndef ROVER_ARDUINO_MAXON_H
#define ROVER_ARDUINO_MAXON_H

#include <Arduino.h>
#include "include/MotorController.h"
#include "include/MotorConfig.h"
namespace motor {

/**
 * \brief Implementation of the Motor Controller Interface
 *
 * Implements the Motor controller interface for the Maxon EC motor controllers.
 *
 * This motor controller does not have a brake pin, so calling brake effectively
 * does the same thing as calling enable.
 */
class MAXON : public MotorController {
public:
    /**
     * \brief Constructor initialises the pin direction and sets safe wait state.
     *
     * \param speedPin PWM speed output pin number
     * \param directionPin Digital output driving the direction of the motor
     * \param enablePin Enable the motor controller
     * \param data1Pin Digital pin for mode selection
     * \param data2Pin Digital pin for mode selection
     * \param feedbackPin Digital input pin indicating the readiness of the motor
     * \param mode Drive mode for the motor controller. Used in the constructor only.
     */
    MAXON(
            uint8_t speedPin,
            uint8_t directionPin,
            uint8_t enablePin,
            uint8_t data1Pin,
            uint8_t data2Pin,
            uint8_t feedbackPin,
            Mode mode
    );

    /**
     * \brief Sets the angular velocity of the wheel
     *
     * \param speed The desired speed output of the motor. The speed should be
     * signed to drive the direction
     */
    virtual void setSpeed(float speed);

    /**
     * \brief Manually set the direction of the motor.
     *
     * Note that the next call to setSpeed will call setDirection again.
     *
     * \param speed Signed speed value. The sign will drive the direction.
     */
    virtual void setDirection(float speed);

    /**
     * \brief Implementation of the brake method.
     *
     * The EC controller does not have a dedicated brake pin
     * so this function drives the enable pin as well
     *
     * \param brk Boolean value true if the wheel should stop.
     */
    virtual void brake(bool brk);

    /**
     * \brief Enables the motor controller with the digital output pin
     *
     * \param en True value enables the motor controller.
     */
    virtual void enable(bool en);

    /**
     * \brief Read the readiness status of the motor controller
     *
     * \return True indicates the motor is operating normally.
     */
    virtual bool getStatus();

private:
    uint8_t mData1Pin;
    uint8_t mData2Pin;
    uint8_t mFeedbackPin;
    uint8_t mSpeedPin;
    uint8_t mDirectionPin;
    uint8_t mEnablePin;

    void setMode(Mode mode);
};
}


#endif //ROVER_ARDUINO_MAXON_H
