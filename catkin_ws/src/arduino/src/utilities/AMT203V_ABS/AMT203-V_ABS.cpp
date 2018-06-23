#include <SPI.h>
#include "AMT203-V_ABS.h"

bool encoder_enabled = true;

AMT_ABS::AMT_ABS(int CSB) {
  _CSB = CSB;
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

uint8_t SPI_T(uint8_t msg, int CSB) { //repetive SPI transmit sequence
  uint8_t msg_temp = 0;
  digitalWrite(CSB, LOW); //select spi device
  msg_temp = SPI.transfer(msg); //send and recieve
  digitalWrite(CSB, HIGH);
  delay(2);
  return (msg_temp);
}

int AMT_ABS::DEG(double * angle) {

  if(!encoder_enabled) {
    return -1;
  }
  uint8_t temp[2];
  uint8_t recieved = 0xA5;
  uint16_t ABSposition = 0;

  SPI.begin();
  digitalWrite(_CSB, LOW);
  SPI_T(0x10, _CSB); //read command

  unsigned long time_begin = millis();
  while(true) {
    recieved = SPI_T(0x00, _CSB);
    if (recieved == 0x10) break;
    if(millis() - time_begin > 10) {
      encoder_enabled = false;
      return -1;
    }
  }


  temp[0] = SPI_T(0x00, _CSB); //recieve MSB
  temp[1] = SPI_T(0x00, _CSB); // recieve LSB
  digitalWrite(_CSB, HIGH);
  SPI.end();

  if (temp[0] != 0xA5 && temp[1] != 0x10) { //outliers
    temp[0] &= 0x0F; //mask out the first 4 bits

    if (temp[0] + temp[1] != 0) { //outliers
      ABSposition = temp[0] << 8; //shift MSB to correct ABSposition
      ABSposition += temp[1]; // add LSB to ABSposition

      if (ABSposition != _ABSposition_last) { //change in position
        _ABSposition_last = ABSposition;

        *angle = ABSposition * 0.08789f; //aprox 360/4096 for deg conversion
        return 0;
      }
    }
  }
}
