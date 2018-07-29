#include <Wire.h>
#include "INA226.h" 
#include <Servo.h> //servo library
#include "AMT203-V_INC.h" //quadrature encoder library, incremental
#include "BDC_DRV8842.h" //brushed driver controller library 
#include "Encoder.h" //download this library from https://github.com/PaulStoffregen/Encoder
//#include <Adafruit_MPL3115A2.h>
#include <Arduino.h>
#include "Science.h"
//#include "SparkFunCCS811.h"

//loadcell pins
#define DOUT_B  24   //DAT_2
#define CLK_B  25    //SCK_2
#define DOUT_T  22   //DAT_2
#define CLK_T 23    //SCK_2

byte gain_A = 128;
byte gain_B = 32;

HX711 scale_TB(DOUT_T, CLK_T, gain_B);
HX711 scale_BB(DOUT_B, CLK_B, gain_B);
HX711 scale_BA(DOUT_B, CLK_B);
HX711 scale_TA(DOUT_T, CLK_T);

Science::Science(){
  
  auger_angle_turned = 0;
  AUGER_DEPTH = 100;
  DRILL_THREAD_DISTANCE = 1; // placeholder

  //setup current monitors
  currentMonitorDRG = new INA226(0x40);
  currentMonitorDrill= new INA226(0x41);
  
  //setup the servos
  servoSetup();

  //setup the encoders
  encoderDrill = new AMT_INC(18, 19);
  encoderCRG = new AMT_INC(3, 2);

  //setup the bdc's
  controllerDrill = new BDC(8, 7, 12, 10);
  controllerCRG = new BDC(9, 6, 13, 11);

  //limit switch setup
  limitSwitchSetup();

  //PAT sensor setup
  baro = Adafruit_MPL3115A2();

  ahtSensor = Adafruit_Si7021();

  //VOCSetup();

}

/*methods to setup the devices*/

void Science::servoSetup(){
  //setup the servos
  servoProbe.write(0);
  servoCmr.write(0);
  servoLidRock.write(0);
  servoLidSoil.write(0);

  servoProbe.attach(0);//not working
  servoCmr.attach(1);
  servoLidRock.attach(4);
  servoLidSoil.attach(5);  
}

void Science::limitSwitchSetup(){
  pinMode(LB_P,INPUT_PULLUP); //LB_P
  pinMode(LB_C,INPUT_PULLUP); //LB_C
  pinMode(LT_P,INPUT_PULLUP); //LT_P
  pinMode(LT_C,INPUT_PULLUP); //LT_C
}

void Science::VOCSetup(){
  vocSensor = new CCS811(CCS811_ADDR);
  //It is recommended to check return status on .begin(), but it is not
  //required.
  CCS811Core::status returnCode = vocSensor->begin(); //initializing sensor
  if (returnCode != CCS811Core::SENSOR_SUCCESS)
  {
    Serial.println(".begin() returned with an error. Program ceased. ");
    while (1); //Hang if there was a problem.
  }
}

void Science::loadcell_setup(){
  scale_TA->set_scale(-455.f); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale_TA->tare(200); //Assuming there is no weight on the scale at start up, reset the scale to 0
  scale_BA->set_scale(-455.f); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale_BA->tare(100); //Assuming there is no weight on the scale at start up, reset the scale to 0
  scale_TB->set_scale(-90.f); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale_TB->tare(200); //Assuming there is no weight on the scale at start up, reset the scale to 0
  scale_BB->set_scale(-90.f); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale_BB->tare(200); //Assuming there is no weight on the scale at start up, reset the scale to 0
  
  scale_TA->read();
  scale_TB->read();
  scale_BA->read();
  scale_BB->read();
}

/*methods called in loop()*/

void Science::driveBDC(BDC *controller, int PWM_val){
  if (controller->FLT()) {
    Serial.println("Fault");
    controller->RST();
  } else {
    while (Serial.available() > 1) {//TODO: change this for serial
      //PWM_val = Serial.parseInt();//debugging purpose
      controller->PWM(PWM_val);
      Serial.println(PWM_val);
    }
  }
}

/*optional method for user*/
void Science::servoTurnTo(Servo servo, int servoPos){
  servo.write(servoPos);
}

uint8_t Science::LimitSwitchPT(){
  if(digitalRead(LT_P)){
    return 0x00;
  }else{
    return 0x01;
  }
}
uint8_t Science::LimitSwitchPB(){
  if(digitalRead(LB_P)){
    return 0x00;
  }else{
    return 0x01;
  }
}
uint8_t Science::LimitSwitchCT(){
  if(digitalRead(LT_C)){
    return 0x00;
  }else{
    return 0x01;
  }
}
uint8_t Science::LimitSwitchCB(){
  if(digitalRead(LB_C)){
    return 0x00;
  }else{
    return 0x01;
  }
}

float Science::windSpeed(){
  float wind_speed = 0, wind_sensor_voltage = 0;
   wind_sensor_voltage = analogRead(A12) * WIND_VOLT_CONSTANT;
   if (wind_sensor_voltage <= WIND_VOLT_MIN){
      wind_speed = 0;
   }
   else {
      wind_speed = (wind_sensor_voltage - WIND_VOLT_MIN)*WIND_SPEED_MAX/(WIND_VOLT_MAX - WIND_VOLT_MIN);
   }
   Serial.print("Wind speed: ");
   Serial.print(wind_speed);
   Serial.println("m/s");

   return wind_speed;
}

int Science::uv(){
  int sensorValue;
  long  sum=0;
  int result;
  for(int i=0;i<1024;i++)// accumulate readings for 1024 times
    {
        sensorValue = analogRead(A14);
        sum = sensorValue + sum;
        delay(2);
    }
    long meanVal = sum/1024;  // get mean value
    result = (meanVal*1000/4.3-83)/21; // get a detailed calculating expression for UV index in schematic files.
    //Serial.print(analogRead(A14));
    //Serial.print("\n");
    Serial.print("The current UV index is:");
    Serial.print(result);
    Serial.print("\n");
    //delay(20);
    return result;
}

int Science::soilH(){
  int humidity_value = analogRead(A15);
    Serial.print("Humidity Level (0-1023): ");
    Serial.println(1023-humidity_value);
    //delay(250);
    return humidity_value;
}

float Science::soilT(){
  int rawADC = analogRead(A13);

  float Vin = (3.293*rawADC)/1024.0;
  
  float tc1 = (Vin - 1.25)/0.003293;

  Serial.print("TC1 voltage:");
  Serial.println(Vin);
  Serial.print("TC1:");
  Serial.println(tc1);

  //delay(1000);
  return tc1;
}


float Science::tempC(){
  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    //delay(250);
    return;
  }

  
  float tempC = baro.getTemperature();
  Serial.print(tempC); Serial.println("*C");

  return tempC;

}
float Science::altm(){
  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    //delay(250);
    return;
  }

  float altm = baro.getAltitude();
  Serial.print(altm); Serial.println(" meters");
  
  return altm;

}
float Science::pascals(){
  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    //delay(250);
    return;
  }

  float pascals = baro.getPressure();
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  // TODO: unit conversion
  Serial.print(pascals/3377); 
  Serial.println(" Inches (Hg)");
  return pascals;

}


float Science::tempC2(){
  if (!ahtSensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    while (true);
  }
  Serial.print("\tTemperature: "); Serial.println(ahtSensor.readTemperature(), 2);
  return ahtSensor.readTemperature();
}

float Science::ah(){
  if (!ahtSensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    while (true);
  }
  Serial.print("Humidity:    "); Serial.print(ahtSensor.readHumidity(), 2);

  return ahtSensor.readHumidity();
}

float * Science::aq(){
  //Check to see if data is ready with .dataAvailable()
  if (vocSensor->dataAvailable())
  {
    //If so, have the sensor read and calculate the results.
    //Get them later
    vocSensor->readAlgorithmResults();
    float results[2];
    Serial.print("CO2[");
    //Returns calculated CO2 reading
    results[0] = vocSensor->getCO2();
    Serial.print(results[0]);
    Serial.print("] tVOC[");
    //Returns calculated TVOC reading
    results[1]= vocSensor->getTVOC();
    Serial.print(results[1]);
    Serial.print("] millis[");
    //Simply the time since program start
    Serial.print(millis());
    Serial.print("]");
    Serial.println();

    return results;
  }

}

float Science::co2(){
  //Check to see if data is ready with .dataAvailable()
  if (vocSensor->dataAvailable())
  {
    //If so, have the sensor read and calculate the results.
    //Get them later
    vocSensor->readAlgorithmResults();
    float result;
    Serial.print("CO2[");
    //Returns calculated CO2 reading
    result = vocSensor->getCO2();
    Serial.print(result);
    Serial.print("] millis[");
    //Simply the time since program start
    Serial.print(millis());
    Serial.print("]");
    Serial.println();

    return result;
  }

}

float Science::voc(){
  //Check to see if data is ready with .dataAvailable()
  if (vocSensor->dataAvailable())
  {
    //If so, have the sensor read and calculate the results.
    //Get them later
    vocSensor->readAlgorithmResults();
    float result;
    Serial.print("VOC[");
    //Returns calculated TVOC reading
    result= vocSensor->getTVOC();
    Serial.print(result);
    Serial.print("] millis[");
    //Simply the time since program start
    Serial.print(millis());
    Serial.print("]");
    Serial.println();

    return result;
  }

}

float Science::scale(HX711 * scale){
  return (scale->read() - scale->get_offset()) / (scale->get_scale());
//  TA = (scale_TA->read() - scale_TA->get_offset()) / (-455.f);
//  TB = (scale_TB->read() - scale_TB->get_offset()) / (-455.f);
//  BA = (scale_BA->read() - scale_BA->get_offset()) / (-90.f);
//  BB = (scale_BB->read() - scale_BB->get_offset()) / (-90.f);
}


