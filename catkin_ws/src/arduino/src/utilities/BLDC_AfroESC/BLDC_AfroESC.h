#ifndef BLDC_h
#define BLDC_h

#include "Arduino.h"
#include "Servo.h"

enum Side{
  LEFT,
  RIGHT
};

class BLDC{
	public:
		BLDC(int pin, Side lr);
		void PWM(int PWM);
	private:
		int _pin;
		Side _lr;
		Servo _motor;
};

#endif
