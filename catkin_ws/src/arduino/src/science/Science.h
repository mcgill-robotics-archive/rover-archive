#ifndef Science_h
#define Science_h

#include <Wire.h>
#include "INA226.h" // current monitor library: include library "INA226-master.zip" by add zip library
#include <Servo.h> //servo library
#include "AMT203-V_INC.h" //quadrature encoder library, incremental
#include "BDC_DRV8842.h" //brushed driver controller library 
#include <Adafruit_MPL3115A2.h>
#include "Adafruit_Si7021.h"
#include "Encoder.h" //download this library from https://github.com/PaulStoffregen/Encoder
#include <Arduino.h>
#include "SparkFunCCS811.h"
#include "HX711.h"



class Science {
  public:
    INA226 * currentMonitorDRG;// initialize current monitor
    INA226 * currentMonitorDrill;
    uint8_t devicesFoundCurrentMonitor = 0; //number of INA226s found
    float current[2] = {0, 0}; // stores current for the CRG and Drill

    Servo servoProbe, servoCmr, servoLidRock, servoLidSoil; //initialize servos

    double auger_angle_turned;
    int AUGER_DEPTH, DRILL_THREAD_DISTANCE; // placeholder
    float WIND_VOLT_CONSTANT = .004882814, WIND_VOLT_MIN = .4, WIND_VOLT_MAX = 2.0, WIND_SPEED_MAX = 32;

    AMT_INC * encoderDrill;//initialize encoders
    AMT_INC * encoderCRG;

    BDC * controllerDrill;//initialize controllers
    BDC * controllerCRG;

    Adafruit_MPL3115A2 baro;// initialize PAT sensor

    Adafruit_Si7021 ahtSensor; // intitialize ATH sensor

    uint8_t CCS811_ADDR = 0x5A;
    CCS811 * vocSensor;
    HX711 * scale_TA;
    HX711 * scale_TB;
    HX711 * scale_BA;
    HX711 * scale_BB;
    float TA, TB, BA, BB;
    
    Science();
    void monitorCurrent();
    void driveBDC(BDC *controller, int PWM_val);
    void servoTurnTo(Servo servo, int servoPos);
    uint8_t LimitSwitchPT();
    uint8_t LimitSwitchPB();
    uint8_t LimitSwitchCT();
    uint8_t LimitSwitchCB();
    float windSpeed();
    int uv();  //TODO: find equation for this
    int soilH(); //TODO: find equation for this
    float soilT(); //TODO: find equation for this
    float tempC();
    float pascals();
    float altm();
    float tempC2();
    float ah();
    float * aq();
    float co2();
    float voc();
    float scale(HX711 * scale);
    

  private:
    void servoSetup();
    void limitSwitchSetup();
    void VOCSetup();
    void loadcell_setup();
    int LB_P = 14, LB_C = 15, LT_P = 16, LT_C = 17;//limit Switch pins/indices
};

#endif
