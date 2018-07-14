#include "BLDC_AfroESC.h"

BLDC::BLDC(int pin, Side lr)
{
    _lr = lr;
	_motor.attach(pin);
	_motor.writeMicroseconds(1500);
  delay(2000);
}

void BLDC::PWM(int us){
	if(_lr == RIGHT){
		us = -us;
	}
	if(us < -255 || us > 255){
		us = 255*(abs(us)/us);
	}
	
	_motor.writeMicroseconds((int) ((450/255)*us + 1460));
	
}
