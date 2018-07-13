//****************************************************************//
// Electrical Division, Mars Rover Team of McGill Robotics
// Authors: Alexandre Coulombe, Adrien Sauvestre, Jeslan Rajendram 
// Winter 2018
// version: 1.0
//  
// AMT203-v encoder code used in Mars Rover's
// Arm system and Drive system 
// Functionality:
//            - Retrieve absolute angular position value
//            - Set encoder 0 location
//****************************************************************//
#ifndef AMT_ABS_h
#define AMT_ABS_h

#include "Arduino.h"

class AMT_ABS{
	public:
		AMT_ABS(int CSB);
    boolean SPI_set_0();
		int DEG(double * angle);
	private:
    boolean encoder_enabled;
		int _CSB;
		uint16_t _ABSposition_last;
};

#endif
