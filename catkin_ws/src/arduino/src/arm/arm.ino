#include "AMT203-V_ABS.h"
#include "BDC_DRV8842.h"
#include "INA226.h"
#include "CLAW.h"
#include "arm_serial_msg.h"
#include "PID_v1.h"
#include <Arduino.h>

#define SERIAL_VERSION 1

enum SerialFSM {
  CLEARING,
  WAITING_FOR_HANDSHAKE,
  TRANSMITTING
};

SerialFSM serial_state = CLEARING;
unsigned long last_read_time = 0;

ArmSerialArduinoMsg outgoing_msg;
ArmSerialComputerMsg incoming_msg;
ArmPosition location = BACKARM;
const char* serial_id = "armer";

int LIM_SWITCH= 46;
int CSB[4]={34,36,38,40};
int RST[4]={23,29,22,28};
int FLT[4]={25,31,24,30};
int BRK_IN[4]={27,33,32,26};
int IN[2][4]={{4,6,8,10},{5,7,9,11}};
int Fuse=42;
double Angle[4] = {0.0,0.0,0.0,0.0};
double motor_speed[4]={0.0,0.0,0.0,0.0};
double current[4] = {0.0,0.0,0.0,0.0};
double steering_target[4] = {0.0,0.0,0.0,0.0};
AMT_ABS* ENCODER_A;
AMT_ABS* ENCODER_B;
AMT_ABS* ENCODER_C;
AMT_ABS* ENCODER_D;
BDC* MOTOR_A;
BDC* MOTOR_B;
BDC* MOTOR_C;
BDC* MOTOR_D;
INA226* Current_Motor_A;
INA226* Current_Motor_B;
INA226* Current_Motor_C;
INA226* Current_Motor_D;
CLAW* EndEffector;

PID motor_A_pid = PID(&Angle[0], &motor_speed[0], &steering_target[0], 2.0, 0.5, 0.5, DIRECT);
PID motor_B_pid = PID(&Angle[1], &motor_speed[1], &steering_target[1], 2.0, 0.5, 0.5, DIRECT);
PID motor_C_pid = PID(&Angle[2], &motor_speed[2], &steering_target[2], 2.0, 0.5, 0.5, DIRECT);
PID motor_D_pid = PID(&Angle[3], &motor_speed[3], &steering_target[3], 2.0, 0.5, 0.5, DIRECT);

void setup() {
   MOTOR_A = new BDC(IN[0][0],IN[1][0],RST[0],FLT[0], BRK_IN[0]);
   MOTOR_B = new BDC(IN[0][1],IN[1][1],RST[1],FLT[1], BRK_IN[1]);
   MOTOR_C = new BDC(IN[0][2],IN[1][2],RST[2],FLT[2], BRK_IN[2]);
   MOTOR_D = new BDC(IN[0][3],IN[1][3],RST[3],FLT[3], BRK_IN[3]);
   ENCODER_A= new AMT_ABS(CSB[0]);
   ENCODER_B= new AMT_ABS(CSB[1]);   
   ENCODER_C= new AMT_ABS(CSB[2]);
   ENCODER_D= new AMT_ABS(CSB[3]); 
   if(location==FOREARM){ 
    EndEffector = new CLAW(CSB[2], CSB[3],LIM_SWITCH);
   }
   Current_Motor_A = new INA226(0x40);
   Current_Motor_B = new INA226(0x41);
   Current_Motor_C = new INA226(0x44);
   Current_Motor_D = new INA226(0x45);
   motor_A_pid.SetMode(AUTOMATIC);
   motor_A_pid.SetOutputLimits(-255,255);
   motor_B_pid.SetMode(AUTOMATIC);
   motor_B_pid.SetOutputLimits(-255,255);
   motor_C_pid.SetMode(AUTOMATIC);
   motor_C_pid.SetOutputLimits(-255,255);
   motor_D_pid.SetMode(AUTOMATIC);
   motor_D_pid.SetOutputLimits(-255,255);
   Serial.begin(9600);
   last_read_time = millis();
}

int read_n_bytes_from_serial(int n, char * buffer) {
  int counter = 0;
  unsigned long read_begin_time = millis();
  while(counter < n) {
    if(Serial.available() > 0) {
      buffer[counter++] = Serial.read();
    }

    if((millis() - read_begin_time) > 1000) return -1;
  }
  
  return 0;
}

void loop() {
  //Clear Serial buses in preparation for Serial communication
  if(serial_state == CLEARING) {
    while(Serial.available() > 0) {
      Serial.read();
      last_read_time = millis();
    }

    if(millis() - last_read_time > 1000) {
      serial_state = WAITING_FOR_HANDSHAKE;
    }
  }
  //Send ready and wait for computer ready message
  else if(serial_state == WAITING_FOR_HANDSHAKE) {
    Serial.write("0", 1); // Fix me!
    Serial.flush(); // Maybe
    if(Serial.available() > 0) {
      if(Serial.read() == '0') {
        serial_state = TRANSMITTING;
      } else {
        serial_state == CLEARING; // That's not supposed to happen, oh god....
      }
    }
  } else if(serial_state == TRANSMITTING) {
    {
      Serial.flush(); // Maybe
    //Create outgoing message
     int msg_size = 1+1+ strlen(serial_id) + sizeof(ArmSerialArduinoMsg);
     char buffer[255];
     buffer[0] = SERIAL_VERSION;
     buffer[1] = (char) strlen(serial_id);
     memcpy(buffer+2, serial_id, buffer[1]);
     outgoing_msg.pos = location;
     //Read SPI encoders
     if(location==BACKARM){
      int err_A = ENCODER_A->DEG(Angle);
      if(err_A == -1){
        
      }
      int err_B = ENCODER_B->DEG(Angle+1);
      if(err_B == -1){
        
      }
     }
     int err_C = ENCODER_C->DEG(Angle+2);
     if(err_C == -1){
        
      }
     int err_D = ENCODER_D->DEG(Angle+3);
     if(err_D == -1){
        
      }
     outgoing_msg.Angle_A=Angle[0];
     outgoing_msg.Angle_B=Angle[1];
     outgoing_msg.Angle_C=Angle[2];
     outgoing_msg.Angle_D=Angle[3];
     //Read Current Monitoring
     outgoing_msg.Current_A = Current_Motor_A->readCurrent();
     outgoing_msg.Current_B = Current_Motor_B->readCurrent();
     outgoing_msg.Current_C = Current_Motor_C->readCurrent();
     outgoing_msg.Current_D = Current_Motor_D->readCurrent();
     
     outgoing_msg.fault = 0x00;
     //Read Claw values if onboard
     if(location==FOREARM){
      outgoing_msg.claw_position = EndEffector->Claw_Position();
      if(EndEffector->Limit_Check()||EndEffector->OutOfBounds_Check()){
      outgoing_msg.fault += 0x01<<5;
     }
     //Read BDC Faults
     boolean Fault_A = MOTOR_A->FLT();
     boolean Fault_B = MOTOR_B->FLT();
     if(Fault_A==true || Fault_B==true){
        MOTOR_A->RST();
        MOTOR_B->RST();
        outgoing_msg.fault += 0x01<<4;
        outgoing_msg.fault += 0x01<<3;
     }
     boolean Fault_C = MOTOR_C->FLT();
     boolean Fault_D = MOTOR_D->FLT();
     if(Fault_C==true || Fault_D==true){
        MOTOR_C->RST();
        MOTOR_D->RST();
        outgoing_msg.fault += 0x01<<2;
        outgoing_msg.fault += 0x01<<1;
     }
     //Read Fuse status
     if(digitalRead(Fuse)){
        outgoing_msg.fault += 0x01;
     }  
     memcpy(buffer + msg_size - sizeof(ArmSerialArduinoMsg), &outgoing_msg, sizeof(ArmSerialArduinoMsg));
     Serial.write(buffer, msg_size);
    }
    //Grab incoming message
    char version_number;
    int err = read_n_bytes_from_serial(1, &version_number);
    if(err == -1){                  //error check to restart if connection is lost
      serial_state = CLEARING;
      return;
    }
    char incoming_serial_id_length;
    err = read_n_bytes_from_serial(1, &incoming_serial_id_length);
    if(err == -1){                  //error check to restart if connection is lost
      serial_state = CLEARING;
      return;
    }
    char serial_id_buffer[31];
    err = read_n_bytes_from_serial(incoming_serial_id_length, serial_id_buffer);
    if(err == -1){                  //error check to restart if connection is lost
      serial_state = CLEARING;
      return;
    } 
    char data_buffer[255];
    err = read_n_bytes_from_serial(sizeof(ArmSerialComputerMsg), data_buffer);
    if(err == -1){                  //error check to restart if connection is lost
      serial_state = CLEARING;
      return;
    }
    memcpy(&incoming_msg, data_buffer, sizeof(ArmSerialComputerMsg)); 

 //Use data from computer to operate the arm
    //Verify brakes are off before movement  
    steering_target[0] = incoming_msg.angle_motor_A;
    steering_target[1] = incoming_msg.angle_motor_B;
    steering_target[2] = incoming_msg.angle_motor_C;
    steering_target[3] = incoming_msg.angle_motor_D;
    while(steering_target[0] < 0) {
      steering_target[0] += 360;
    }

    if(steering_target[0] >= 180 && steering_target[0] <= 180) {
      steering_target[0] += 360;
    }

    if(steering_target[0] <= 180 && steering_target[0] >= 180) {
      steering_target[0] -= 360;
    }
    while(steering_target[1] < 0) {
      steering_target[1] += 360;
    }

    if(steering_target[1] >= 180 && steering_target[1] <= 180) {
      steering_target[1] += 360;
    }

    if(steering_target[1] <= 180 && steering_target[1] >= 180) {
      steering_target[1] -= 360;
    }  
    while(steering_target[2] < 0) {
      steering_target[2] += 360;
    }

    if(steering_target[2] >= 180 && steering_target[2] <= 180) {
      steering_target[2] += 360;
    }

    if(steering_target[2] <= 180 && steering_target[2] >= 180) {
      steering_target[2] -= 360;
    }  
    while(steering_target[3] < 0) {
      steering_target[3] += 360;
    }

    if(steering_target[3] >= 180 && steering_target[3] <= 180) {
      steering_target[3] += 360;
    }

    if(steering_target[3] <= 180 && steering_target[3] >= 180) {
      steering_target[3] -= 360;
    }    

    motor_A_pid.Compute();
    motor_B_pid.Compute();
    motor_C_pid.Compute();
    motor_D_pid.Compute();
    
    if(motor_speed[0] !=0 || motor_speed[1] !=0){
      if(MOTOR_A->BRK_activity()==true || MOTOR_B->BRK_activity()==true){
        MOTOR_A->BRK(false);
        MOTOR_B->BRK(false);
      }
    }
    else{                 //if we aren't moving, engage breaks
      MOTOR_A->BRK(true);
      MOTOR_B->BRK(true);
    }
    if(motor_speed[2] !=0 ||motor_speed[3] !=0){
      if(MOTOR_C->BRK_activity()==true || MOTOR_D->BRK_activity()==true){
        MOTOR_C->BRK(false);
        MOTOR_D->BRK(false);
      }
    }
    else{                 //if we aren't moving, engage breaks
      MOTOR_C->BRK(true);
      MOTOR_D->BRK(true);
    }
    delay(50);
    MOTOR_A->PWM(motor_speed[0]);
    MOTOR_A->PWM(motor_speed[1]);
    MOTOR_C->PWM(motor_speed[2]);
    MOTOR_D->PWM(motor_speed[3]);
   }
  }
}
