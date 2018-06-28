//****************************************************************//
// Electrical Division, Mars Rover Team of McGill Robotics
// Authors: Alexandre Coulombe, John Omotayo, Raymond So 
// Winter 2018
// version: 1.0
//  
// INA226 code used in the Mars Rover's
// Arm system and Science system
// Functionality:
//          - monitor the curret going through the motor
//          - monitor the voltage output of the DRV8842
//          - monitor the voltage drop at the shunt resistor
//****************************************************************//

#include "INA226.h"
#include <Wire.h>

//****************************************************************//
// Constructor for the INA226 chip
// The chip requires its INA226 address, which can be changed
// by connecting the A0 and A1 pins in certain ways (view datasheet)
//****************************************************************//
INA226::INA226(uint8_t address){
  _address = address;
  Wire.begin();
}

//****************************************************************//
// Read the shunt voltage from the INA226 ShuntVoltage register
// Write on the I2C data line the address of the chip of interest
// Write to the ShuntVoltage register
// Read the two bytes of data sent from the register
// Return the interpreted data
//****************************************************************//
float INA226::readShuntVoltage() {
  int16_t returnData;
  Wire.beginTransmission(_address);
  Wire.write(Shunt_Voltage_Register);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(_address, (uint8_t)2);
  if(Wire.available()>1){
    returnData = ((Wire.read()<<8)| Wire.read());
  }
  return ((float)returnData*Shunt_LSB/1000); //(mV)
}

//****************************************************************//
// Read the bus voltage from the INA226 BusVoltage register
// Write on the I2C data line the address of the chip of interest
// Write to the BusVoltage register
// Read the two bytes of data sent from the register
// Return the interpreted data
//****************************************************************//
float INA226::readBusVoltage() {
  int16_t returnData;
  Wire.beginTransmission(_address);
  Wire.write(Bus_Voltage_Register);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(_address, (uint8_t)2);
  if(Wire.available()>1){
    returnData = ((Wire.read()<<8)| Wire.read());
  }
  return ((float)returnData*Bus_LSB/1000);   //(V)
}

//****************************************************************//
// Read the current from the INA226
// Aquire shunt voltage from ShuntVoltage register
// Return the interpreted data by I=V/R
//****************************************************************//
float INA226::readCurrent() {
  return (readShuntVoltage()/5); //(mA), 5 is the best calibration for accurate current readings for the BDC
}

