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
 
#include "BDC_DRV8842.h"

//****************************************************************//
// Constructor for a BDC object
// The BDC requires the PWM pins used on the arduino as In1 and In2,
// the pin used to Reset the DRV8842, the pin used to monitor the 
// Fault, and the pin going to the Gate of the CSD16340 
//****************************************************************//
BDC::BDC(int In1, int In2, int nRST, int nFLT, int Brake){
  pinMode(In1, OUTPUT);
  _In1 = In1;
   pinMode(In2, OUTPUT);
  _In2 = In2;
  pinMode(nRST, OUTPUT);
  _nRST = nRST;
  pinMode(nFLT, INPUT);
  _nFLT = nFLT;
  pinMode(Brake, OUTPUT);
  _BRK = Brake;
  BRK(true);
  RST();
}

//****************************************************************//
// Send PWM signals to the DRV8842 motor controller
// The function take in a PWM signal from the user and sends it
// to the DRV8842 for it to send to corresponding ratio to the
// motor, with the correct direction and boundary check 
//****************************************************************//
void BDC::PWM(int PWM_val){
    if(PWM_val < -255){                 //Boundary check for min value
      PWM_val = -255;
    }
    else if (PWM_val > 255){            //Boundary check for max value
      PWM_val = 255;
    }
   
    if(PWM_val > 0){                    //Sets the motor to spin in one
      analogWrite(_In1, abs(PWM_val));  //direction at PWM specified
      digitalWrite(_In2, LOW);
    }
    else if (PWM_val < 0){              //Sets the motor to spin in the
      digitalWrite(_In1, LOW);          //other direction at the PWM specified
      analogWrite(_In2, abs(PWM_val));
    }
    else{                               //Occurs only at 0 and sets the motor idle
      digitalWrite(_In1, LOW);
      digitalWrite(_In2, LOW);
    }
}

//****************************************************************//
// Control the activation of the motor's brake (only for Arm motors)
// Takes in a boolean (true or false) to engage or disengage the brake
// %WARNING% : at 50ms must be allocated to disengage the brake
// before a PWM can be sent (prevent blocks and motor damage)
//****************************************************************//
void BDC::BRK(boolean active){
  if(active == true){                   //When the pin is set low,
    digitalWrite(_BRK, LOW);            //the MOSFET acts as an open 
  }                                     //and the brake engages (no power)
  else{                                 //When the pin is set high,
    digitalWrite(_BRK, HIGH);           //the MOSFET acts as a short
  }                                     //and the brake disengages (has power)
}

//****************************************************************//
// Verify the brake activity of the Motor 
//****************************************************************//
boolean BDC::BRK_activity(){
  return !digitalRead(_BRK);            //Returns the opposite logic of the pin (brake on, pin low)
}
//****************************************************************//
// Reset the DRV8842 
//****************************************************************//
void BDC::RST(){
  digitalWrite(_nRST , LOW);
  digitalWrite(_nFLT , HIGH); 
  delay(500);
  digitalWrite(_nRST, HIGH);
  delay(250);
}

//****************************************************************//
// Read the DRV8842 fault state 
// Fault pin state: HIGH = Good, LOW = fault occured
//****************************************************************//
int BDC::FLT(){
  // READ the fault pin
  return !(digitalRead(_nFLT));
}
