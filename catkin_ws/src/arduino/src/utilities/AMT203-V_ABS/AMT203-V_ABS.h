#ifndef AMT_ABS_h
#define AMT_ABS_h

#include "Arduino.h"

class AMT_ABS{
	public:
		AMT_ABS(int CSB);
		int DEG();
	private:
		int _CSB;
		float _deg;
		uint16_t _ABSposition;
		uint16_t _ABSposition_last;
};

#endif
