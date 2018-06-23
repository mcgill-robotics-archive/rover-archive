//****************************************************************//
// Electrical Division, Mars Rover Team of McGill Robotics
// Authors: Alexandre Coulombe, Adrien Sauvestre, Jeslan Rajendram 
// Winter 2018
// version: 1.0
//  
// AMT203-v encoder code used in Mars Rover's
// Arm system and Drive system 
// Functionality:
//            - Retrieve absolute angular position value through SPI
//            - Set encoder 0 location
//****************************************************************//

#include  <SPI.h>
#include  "AMT203-V_ABS.h"

//****************************************************************//
// Construtor for an AMT203-v object
// The encoder requires only the digital pin used as Slave Select
//****************************************************************//
AMT_ABS::AMT_ABS(int CSB) {
  _CSB = CSB;
  _deg = 0.0f;
  _ABSposition = 0;
  _ABSposition_last = 0;
  pinMode(_CSB, OUTPUT);
  digitalWrite(_CSB, HIGH);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  delay(2000);
  SPI.end();
}

//****************************************************************//
// Transmits message through the SPI protocol to selected device
//****************************************************************//
uint8_t SPI_T(uint8_t msg, int CSB) { //repetive SPI transmit sequence
  uint8_t msg_temp = 0;
  digitalWrite(CSB, LOW); //select spi device
  msg_temp = SPI.transfer(msg); //send and recieve
  digitalWrite(CSB, HIGH);
  delay(2);
  return (msg_temp);
}

//****************************************************************//
// Resets the encoders 0 position to its current position
//****************************************************************//
boolean AMT_ABS::SPI_set_0(){   //set the encoders current position to new reference zero degrees
  SPI.begin();
  SPI_T(0x70, _CSB);
  unsigned long currentmillis = millis();
  unsigned long startmillis = currentmillis;
  while ((currentmillis-startmillis)<100) { //check if encoder is still working
    uint8_t received = SPI_T(0x00, _CSB);
    if (received == 0x80)break;
    currentmillis = millis();
  }
  SPI.end();
  if((currentmillis-startmillis)>=100){
    return false;                           //false signifies that the encoder did not set itself to zero
  }
  return true;                              //true signifies that the encoder succesfully set itself to zero
}//The encoder must be turned off and on again for the encoder to operate from the new zero position

//****************************************************************//
// Retrieve the encoders absolute angular position to the arduino
//****************************************************************//
float AMT_ABS::DEG() {
  uint8_t temp[2];
    
  uint8_t recieved = 0xA5;
  _ABSposition = 0;

  SPI.begin();
  digitalWrite(_CSB, LOW);
  SPI_T(0x10, _CSB); //read command
  unsigned long currentmillis = millis();
  unsigned long startmillis = currentmillis;
  while ((currentmillis-startmillis)<100) { //check if encoder is still working
    recieved = SPI_T(0x00, _CSB);
    if (recieved == 0x10)break;
    currentmillis = millis();
  }
  if((currentmillis-startmillis)>=100){
    return -1; //we lost the encoder
  }

  temp[0] = SPI_T(0x00, _CSB); //recieve MSB
  temp[1] = SPI_T(0x00, _CSB); // recieve LSB
  digitalWrite(_CSB, HIGH);
  SPI.end();

  if (temp[0] != 0xA5 && temp[1] != 0x10) { //outliers
    temp[0] &= 0x0F; //mask out the first 4 bits

    if (temp[0] + temp[1] != 0) { //outliers
      _ABSposition = temp[0] << 8; //shift MSB to correct ABSposition
      _ABSposition += temp[1]; // add LSB to ABSposition

      if (_ABSposition != _ABSposition_last) { //change in position
        _ABSposition_last = _ABSposition;
        _deg = _ABSposition * 0.08789; //aprox 360/4096
        return _deg;
      }
    }
  }
}
