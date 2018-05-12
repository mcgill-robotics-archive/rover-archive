#include "BDC_DRV8842.h"

BDC::BDC(int In1, int In2, int nRST, int nFLT){
  pinMode(In1, OUTPUT);
  _In1 = In1;
   pinMode(In2, OUTPUT);
  _In2 = In2;
  pinMode(nRST, OUTPUT);
  _nRST = nRST;
  pinMode(nFLT, INPUT);
  _nFLT = nFLT;

  RST();
}

void BDC::PWM(int PWM_val){
	if(PWM_val < -255)
	{
	  PWM_val = -255;
	}
	else if (PWM_val > 255)
	{
		PWM_val = 255;
	}
 
	if(PWM_val > 0){
		analogWrite(_In1, abs(PWM_val));
		digitalWrite(_In2, LOW);
	}
	else if (PWM_val < 0){
		digitalWrite(_In1, LOW);
		analogWrite(_In2, abs(PWM_val));
	}
	else{
		digitalWrite(_In1, LOW);
		digitalWrite(_In2, LOW);
	}
}

void BDC::RST(){
  digitalWrite(_nRST , LOW);
  digitalWrite(_nFLT , HIGH); 
  delay(500);
  digitalWrite(_nRST, HIGH);
  delay(250);
}

int BDC::FLT(){
  // READ the fault pin
  return !(digitalRead(_nFLT));
}
