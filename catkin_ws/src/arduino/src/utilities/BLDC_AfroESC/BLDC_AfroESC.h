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
		BLDC(int pin, Side lr, int zero);
		void PWM(int PWM);
	private:
		Side _lr;
		Servo _motor;
        int _zero;
};

#endif
