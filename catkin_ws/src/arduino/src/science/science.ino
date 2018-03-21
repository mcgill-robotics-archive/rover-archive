#include <Wire.h>
#include <INA226.h> // current monitor library
#include <Servo.h> //servo library
#include "AMT203-V_INC.h" //quadrature encoder library, incremental
#include "BDC_DRV8842.h" //brushed driver controller library 

INA226_Class INA226;// initialize current monitor
uint8_t devicesFoundCurrentMonitor = 0; //number of INA226s found
float current[2] = {0,0};// stores current for the CRG and Drill 

Servo servoProbe,servoCmr,servoLidRock,servoLidSoil;  //initialize servos 
int SERVO_PROBE = 0, SERVO_CAMERA = 1, SERVO_ROCK_LID = 2, SERVO_SOIL_LID = 3;//servo indices

int auger_angle_turned = 0;
int AUGER_DEPTH = 100, DRILL_THREAD_DISTANCE = 1; // placeholder
AMT_INC * encoderDrill;//initialize encoders
AMT_INC * encoderCRG;
int MOTOR_DRILL = 0, MOTOR_CRG = 1;//motor indices 
int drillSpeed = 10, crgSpeed = 10;
//int us;


BDC * controllerDrill;//initialize controllers
BDC * controllerCRG;


void setup() {
  
    Serial.begin(9600);
    //setup current monitors
    currentMonitorSetup();
    
    //setup the servos
    servoSetup();

    //setup the encoders
    encoderDrill = new AMT_INC(18, 19);
    encoderCRG = new AMT_INC(3, 2);

    //setup the bdc's
    controllerDrill = new BDC(8, 7, 12, 10);
    controllerCRG = new BDC(9, 6, 13, 11);

    //limit switch setup
    pinMode(limitSwitch1,INPUT);
 
   
}

void loop() {
  // put your main code here, to run repeatedly:
  monitorCurrent();
  monitorBDC(MOTOR_DRILL,drillSpeed);
  monitorBDC(MOTOR_CRG,crgSpeed);
  //monitorMotor(MOTOR_DRILL);// WIP
  //monitorMotor(MOTOR_CRG);
  
  auger_angle_turned += (encoderDrill->delta());
  if (auger_angle_turned>= (AUGER_DEPTH/DRILL_THREAD_DISTANCE*360)){
    //stop auger;
  }
   
  int servoAngle = Serial.parseInt();
  servoTurnTo(0, servoAngle);
  servoTurnTo(1, servoAngle);
  servoTurnTo(2, servoAngle);
  servoTurnTo(3, servoAngle);
  
  
  
}

/*methods to setup the devices*/
void currentMonitorSetup(){
  devicesFoundCurrentMonitor = INA226.begin(5,4000); //set expected Amps and resistor (microOhms)                        
  INA226.setAveraging(4); //average each reading n-times
  INA226.setShuntConversion(7); //maximum conversion time 8.244ms
  INA226.setMode(INA_MODE_CONTINUOUS_SHUNT); //shunt measured continuously
}

void servoSetup(){
    //setup the servos
    servoProbe.write(0);
    servoCmr.write(0);
    servoLidRock.write(0);
    servoLidSoil.write(0);

    servoProbe.attach(0);
    servoCmr.attach(1);
    servoLidRock.attach(4);
    servoLidSoil.attach(5);  
}


/*methods called in loop()*/
void monitorCurrent(){
    for (uint8_t i=0;i<devicesFoundCurrentMonitor;i++){ //loop through all devices found   
    /*print message*/
    Serial.print("Device: ");                              
    Serial.println(i+1);
    Serial.print("Amperage: ");                                                   
    Serial.print((float)INA226.getBusMicroAmps()/1000.0f,3); // Convert to milliamp              
    Serial.println("mA");
    /*store the current values in to the current array*/
    current[i] = (float)INA226.getBusMicroAmps()/1000.0f,3;
  }
  delay(2000); // modify the delay length
}

void monitorBDC(int motorIndex, int PWM_val){
  BDC * controller;
  switch(motorIndex)
  {
    case 0: controller = controllerDrill; break;
    case 1: controller = controllerCRG; break;
    default: Serial.println("Invalid motor name");
    
  }
  if (controller->FLT()) {
    Serial.println("Fault");
    controller->RST();
  } else {
    while (Serial.available() > 1) {
      //PWM_val = Serial.parseInt();
      controller->PWM(PWM_val);
      Serial.println(PWM_val);
    }
  }
}

/*optional method for user*/
void servoTurnTo(int servoIndex, int servoPos){
  switch (servoIndex)
  {
    case 0:
      Serial.print("Probe");
      Serial.print(" Servo rotating to ...");
      Serial.println(servoPos);
      servoProbe.write(servoPos);
      break;
    case 1:
      Serial.print("Camera");
      Serial.print(" Servo rotating to ...");
      Serial.println(servoPos);
      servoCmr.write(servoPos);
      break;
    case 2:
      Serial.print("Rock Lid");
      Serial.print(" Servo rotating to ...");
      Serial.println(servoPos);
      servoLidRock.write(servoPos);
      break;
    case 3:
      Serial.print("Soil Lid");
      Serial.print(" Servo rotating to ...");
      Serial.println(servoPos);
      servoLidSoil.write(servoPos);
      break;
    default: Serial.println("Invalid Servo Name");
  }
}

//
//void motorSpeed(int motorIndex){
//  AMT_INC * encoder;
//  switch (motorIndex)
//  {
//    case 0:

//      break;
//    case 1:

//      break;
//    default: Serial.println("Invalid Motor Name");
//  
//  
//  newposition = encoder0Pos;
//  newtime = (0.001*millis());
//  vel = (newposition-oldposition)/(newtime-oldtime);
//  Serial.print ("\n speed = ");
//  Serial.print (vel);
//  oldposition = newposition;
//  oldtime = newtime;
//  delay(1000);
//}





