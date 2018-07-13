#include "INA3221.h"
#include "power_serial_msg.h"
#include <Servo.h>

#define SERIAL_VERSION 1
enum SerialFSM {
  CLEARING,
  WAITING_FOR_HANDSHAKE,
  TRANSMITTING
};

SerialFSM serial_state = CLEARING;
unsigned long last_read_time = 0;

PowerSerialArduinoMsg outgoing_msg;
PowerSerialComputerMsg incoming_msg;
const char* serial_id = "POWER";

int LTC_E2 = 7; //Control Battery 2 for load sharing
int LTC_E1 = 8; //Control Battery 1 for load sharing
int Fuse_Drive1= A0; //Fuse Detect Drive 1
int Fuse_Drive2= A1; //Fuse Detect Drive 2
int Fuse_Arm = A2; //Fuse Detect Arm
int Fuse_Science = A3; //Fuse Detect Science
int Fuse_5V = A4; //Fuse Detect 5V
int Fuse_12V = A5; //Fuse Detect 12V

// LIPO_Battery_1 Channel is 1;
// LIPO_Battery_2 Channel is 2;
// Load_Sharing Channel is 3;
INA3221 V_monitor;
float BusVoltage[3] = {0,0,0};

Servo pitch;
Servo tilt;

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

void LoadShare(char activity){
  switch (activity){
    case 1:
      digitalWrite(LTC_E2, LOW);
      digitalWrite(LTC_E1, HIGH);
      break;
    case 2:
      digitalWrite(LTC_E2, HIGH);
      digitalWrite(LTC_E1, LOW);
      break;
//    case 3:                             //Never make this case happen
//      digitalWrite(LTC_E2, HIGH);       //This cuts both batteries
//      digitalWrite(LTC_E1, HIGH);       //Which in consequence kills everything
//      break;
    default:
      digitalWrite(LTC_E1,LOW);
      digitalWrite(LTC_E2,LOW);
      break;
  }
}

char Share_state(){
  char state;
  if(digitalRead(LTC_E1)==false &&digitalRead(LTC_E2)==false){
     state = 0;
  }
  else if(digitalRead(LTC_E1)==true &&digitalRead(LTC_E2)==false){
     state = 1;
  }
  else if(digitalRead(LTC_E1)==false &&digitalRead(LTC_E2)==true){
     state = 2;
  }
  return state;
}
uint8_t Fuse_pop(int fuse){
  if(!digitalRead(fuse)){
    return 0x01;
  }
  return 0x00;
}

void setup() {
  V_monitor.begin();
  pinMode(LTC_E2, OUTPUT);
  pinMode(LTC_E1, OUTPUT);
  pinMode(Fuse_Drive1, INPUT);
  pinMode(Fuse_Drive2, INPUT);
  pinMode(Fuse_Arm, INPUT);
  pinMode(Fuse_Science, INPUT);
  pinMode(Fuse_12V, INPUT);
  pitch.attach(5);
  tilt.attach(6);
  Serial.begin(9600);
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
     int msg_size = 1+1+ strlen(serial_id) + sizeof(PowerSerialArduinoMsg);
     char buffer[255];
     buffer[0] = SERIAL_VERSION;
     buffer[1] = (char) strlen(serial_id);
     memcpy(buffer+2, serial_id, buffer[1]);
     outgoing_msg.fuse = 0x00;
     outgoing_msg.fuse += Fuse_pop(Fuse_Drive1)<<5;              //change these for the char bit mapping
     outgoing_msg.fuse += Fuse_pop(Fuse_Drive2)<<4;
     outgoing_msg.fuse += Fuse_pop(Fuse_Arm)<<3;
     outgoing_msg.fuse += Fuse_pop(Fuse_Science)<<2;
     outgoing_msg.fuse += Fuse_pop(Fuse_5V)<<1;
     outgoing_msg.fuse += Fuse_pop(Fuse_12V);
     outgoing_msg.share_state = Share_state();
     memcpy(buffer + msg_size - sizeof(PowerSerialArduinoMsg), &outgoing_msg, sizeof(PowerSerialArduinoMsg));
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
    err = read_n_bytes_from_serial(sizeof(PowerSerialComputerMsg), data_buffer);
    if(err == -1){                  //error check to restart if connection is lost
      serial_state = CLEARING;
      return;
    }
    memcpy(&incoming_msg, data_buffer, sizeof(PowerSerialComputerMsg));
    LoadShare(incoming_msg.power_state);
    pitch.write(incoming_msg.angle_pitch);
    tilt.write(incoming_msg.angle_tilt);
   }
}  
