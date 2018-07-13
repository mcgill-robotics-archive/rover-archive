#include "BLDC_AfroESC.h"

BLDC::BLDC(int pin, Side lr, int zero)
{
    _lr = lr;
    _zero = zero;
	_motor.attach(pin);
	_motor.writeMicroseconds(_zero);
  delay(2000);
}

void BLDC::PWM(int us){
	if(_lr == RIGHT){
		us = -us;
	}
	if(us < -255 || us > 255){
		us = 255*(abs(us)/us);
	}

	_motor.writeMicroseconds((int) ((750/255)*us + _zero));

}
