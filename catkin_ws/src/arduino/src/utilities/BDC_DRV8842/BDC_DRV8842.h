#ifndef BDC_h
#define BDC_h

#include "Arduino.h"

class BDC{
  public:
    BDC(uint8_t In1, uint8_t In2, uint8_t nRST, uint8_t nFLT);
    void PWM(int PWM_val);
    void RST();
	int FLT();
  
  private: 
    uint8_t _In1;
	uint8_t _In2;
    uint8_t _nRST;
	uint8_t _nFLT;
};

#endif
