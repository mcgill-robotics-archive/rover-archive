#include <Arduino.h>
#include "Encoder.h" //download this library from https://github.com/PaulStoffregen/Encoder
#include "AMT203-V_INC.h"

AMT_INC::AMT_INC(int pin1, int pin2){
	_pin1 = pin1;
	_pin2 = pin2;
  _encoder = new Encoder(_pin1, _pin2);
	_ticks = 0;
	_angle = 0;
	_angle_old = 0;
}

double AMT_INC::delta(){
	_ticks = (_encoder->read())%4096;
	_angle = (360.0/4096.0)*_ticks;
	if (_angle < 0){
		_angle = _angle + 360;
	}
	double delta_angle = (_angle - _angle_old);
  delta_angle = ((int) delta_angle)%360 + (delta_angle - ((int) delta_angle));
  _angle_old = _angle;
  if(delta_angle < -300){
    delta_angle += 360;
  }
  if(delta_angle > 300){
    delta_angle -= 360;
  }
	return delta_angle;
}
