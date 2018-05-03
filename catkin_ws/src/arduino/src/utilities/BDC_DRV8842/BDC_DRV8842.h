#ifndef BDC_h
#define BDC_h

#include "Arduino.h"

class BDC{
  public:
    BDC(int In1, int In2, int nRST, int nFLT);
    void PWM(int PWM_val);
    void RST();
	  int FLT();
  
  private: 
    int _In1;
	  int _In2;
	  int _nRST;
	  int _nFLT;
};

#endif
