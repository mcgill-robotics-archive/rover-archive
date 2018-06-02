#ifndef AMT_ABS_h
#define AMT_ABS_h

#include "Arduino.h"

class AMT_ABS{
	public:
		AMT_ABS(int CSB);
		int DEG(float * angle);
	private:
		int _CSB;
		uint16_t _ABSposition_last;
};

#endif
