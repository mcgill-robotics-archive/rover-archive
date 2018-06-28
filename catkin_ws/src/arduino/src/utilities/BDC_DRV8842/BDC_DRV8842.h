//****************************************************************//
// Electrical Division, Mars Rover Team of McGill Robotics
// Authors: Alexandre Coulombe, Chelsea Myers-Colet, Jeslan Rajendram 
// Winter 2018
// version: 1.0
//  
// BDC board code used in the Mars Rover's
// Arm system, Science system, and Drive system
// Functionality:
//          - DRV8842 motor control
//          - DRV8842 fault monitoring
//          - DRV8842 IC reset
//          - CSD16340 MOSFET control
//****************************************************************//

#ifndef BDC_h
#define BDC_h

#include "Arduino.h"

class BDC{
  public:
    BDC(int In1, int In2, int nRST, int nFLT, int BRK);
    void PWM(int PWM_val);
    void RST();
    int FLT();
    void BRK(boolean active);
    boolean BRK_activity();
  
  private: 
    int _In1;
    int _In2;
    int _nRST;
    int _nFLT;
    int _BRK;
};

#endif
