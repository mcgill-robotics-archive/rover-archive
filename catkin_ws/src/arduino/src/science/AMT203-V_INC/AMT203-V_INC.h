#ifndef AMT_INC_h
#define AMT_INC_h

#include "Encoder.h"

class AMT_INC{
	public:
		AMT_INC(int pin1, int pin2);
		double delta();
	private:
		int _pin1;
		int _pin2;
		int _ticks;
		double _angle;
		double _angle_old;
    Encoder * _encoder;
};

#endif
