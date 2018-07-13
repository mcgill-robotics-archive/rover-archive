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

#ifndef INA226_h
#define INA226_h

#include "Arduino.h"

class INA226{
  public:
    INA226(uint8_t address);
    float readShuntVoltage();
    float readBusVoltage();
    float readCurrent();
  private:
    uint8_t _address;
    float Shunt_LSB = 2.5;      //in microV
    float Bus_LSB = 1.25;       //in mV
    //Address location of all INA226 Registers on the chip
    const uint8_t Configuration_Register = 0x00; //All-register reset, shunt voltage and bus voltage ADC conversion times and averaging, operating mode
    const uint8_t Shunt_Voltage_Register = 0x01; //Shunt voltage measurement data
    const uint8_t Bus_Voltage_Register   = 0x02; //Bus voltage measurement data
    const uint8_t Power_Voltage_Register = 0x03; //Contains the value of the calculated power being delivered to the load
    const uint8_t Current_Register       = 0x04; //Contains the value of the calculated current flowing through the shunt resistor
    const uint8_t Calibration_Register   = 0x05; //Sets full-scale range and LSB of current and power measurements. Overall system calibration
    const uint8_t Mask_Enable_Register   = 0x06; //Alert configuration and Conversion Ready flas
    const uint8_t Alert_Limit_Register   = 0x07; //Contains the limit value to compare to the selected Alert function
    const uint8_t Manufacturer_ID_Register=0xFE; //Contains unique manufacturer identification number
    const uint8_t Die_ID_Register        = 0xFF; //Contains unique die identification number
};

#endif

