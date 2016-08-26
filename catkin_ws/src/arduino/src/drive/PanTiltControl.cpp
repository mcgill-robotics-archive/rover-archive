//
// Created by Malcolm Watt on 26/08/16.
//

#include "PanTiltControl.h"

pan_tilt_control::PanTiltControl::PanTiltControl(uint8_t panPin, uint8_t tiltPin) {
    panServoPin = panPin;
    tiltServo->attach(tiltPin);
}

pan_tilt_control::PanTiltControl::~PanTiltControl() {
    delete tiltServo;
}

void pan_tilt_control::PanTiltControl::setPanSpeed(int speed) {
    // Implicitly, since the constant is the 0 speed point, if we have a negative speed then it reverses and vice versa
    unsigned int pulseWidth = STOP_PAN_PW + speed;

    digitalWrite(panServoPin, HIGH);

    delayMicroseconds(pulseWidth);

    digitalWrite(panServoPin, LOW);
}

void pan_tilt_control::PanTiltControl::setTiltSpeed(int speed) {
    // Uses position control to mimic speed control
    currentPanPosition += speed;
    tiltServo->write(currentPanPosition);
}