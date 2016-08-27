//
// Created by Malcolm Watt on 26/08/16.
//

#include "PanTiltControl.h"

pan_tilt_control::PanTiltControl::PanTiltControl(uint8_t panPin, uint8_t tiltPin) {
    panServo.attach(panPin);
    tiltServo.attach(tiltPin);
}

pan_tilt_control::PanTiltControl::~PanTiltControl() {}

void pan_tilt_control::PanTiltControl::setPanSpeed(int speed) {
    // Implicitly, since the constant is the 0 speed point, if we have a negative speed then it reverses and vice versa
    int pulseWidth = STOP_PAN_PW + speed;

    panServo.writeMicroseconds(pulseWidth);
}

void pan_tilt_control::PanTiltControl::setTiltSpeed(int speed) {
    // Uses position control to mimic speed control
    currentPanPosition += speed;
    tiltServo.write(currentPanPosition);
}