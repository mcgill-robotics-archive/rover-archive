// @TODO:
// Clean up
// Add timeout and reset if not receiving messages from computer.

#include<Arduino.h>
#include<string.h>

#include "drive_serial_msgs.h"

#include <Servo.h>
#include "Encoder.h"

#include "BLDC_AfroESC.h"
#include "BDC_DRV8842.h"
#include "AMT203-V_ABS.h"

#define SERIAL_VERSION 1

const char * serial_id = "drive";

enum SerialFSM {
  CLEARING,
  WAITING_FOR_HANDSHAKE,
  TRANSMITTING
};

Side roverSide;
SerialFSM serial_state = CLEARING;
unsigned long last_read_time = 0;
Encoder * Enc;

BDC * brushedMotor;
AMT_ABS * absEncoder;
BLDC * brushlessMotor1;
BLDC * brushlessMotor2;

//Variables for distance calculation using incremental encoder data:
short oldTheta = 0;
short newTheta = 0;
double dist = 0.0;
const float Pi = 3.14159;

DrivePosition pos = FRONT_LEFT;
double steering_angle= 10 ;
short distance; // I WANT SPEED 1&2 FEEDBACK HERE
char fault; // boolean
char fuse; // boolean

//Define roverSide, topics and incremental encoder pinout according to board location on Rover:
DrivePosition location = FRONT_LEFT; //Manually set according to board location on Rover
DriveSerialArduinoMsg outgoing_message;
DriveSerialComputerMsg incoming_msg = {};

void setup(){
  incoming_msg.pos = 0;
  Serial.begin(9600);
  last_read_time = millis();

  if (location == FRONT_RIGHT) {
    roverSide = RIGHT;
    Enc = new Encoder(3, 2);  /* Rignt wheel: (B, A),  Left wheel: (A, B)*/
  }

  else if (location == FRONT_LEFT) {
    roverSide = LEFT;
    Enc = new Encoder(3, 2);  /* Rignt wheel: (B, A),  Left wheel: (A, B)*/
  }

  else if (location == BACK_RIGHT) {
    roverSide = RIGHT;
    Enc = new Encoder(3, 2);  /* Rignt wheel: (B, A),  Left wheel: (A, B)*/
  }

  else if (location == BACK_LEFT) {
    roverSide = LEFT;
    Enc = new Encoder(2, 3);  /* Rignt wheel: (B, A),  Left wheel: (A, B)*/
  }

  brushedMotor = new BDC(11, 13, 7, 8);
  absEncoder = new AMT_ABS(SS);

  brushlessMotor1 = new BLDC(6, roverSide);
  brushlessMotor2 = new BLDC(5, roverSide);

  pinMode(4, OUTPUT);

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

void loop(){
  if(serial_state == CLEARING) {
    while(Serial.available() > 0) {
      Serial.read();
      last_read_time = millis();
    }

    if(millis() - last_read_time > 1000) {
      serial_state = WAITING_FOR_HANDSHAKE;
    }
  } else if(serial_state == WAITING_FOR_HANDSHAKE) {
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
      int msg_size = 1 + 1 + strlen(serial_id) + sizeof(DriveSerialArduinoMsg); // Constant
      char buffer[255];
      buffer[0] = SERIAL_VERSION;
      buffer[1] = (char) strlen(serial_id);
      memcpy(buffer + 2, serial_id, buffer[1]);

      //Side: to be set for testing
      outgoing_message.pos = location;

      //Fault Detection:
      outgoing_message.fault = brushedMotor -> FLT();

      //Fuse Detection:
      outgoing_message.fuse = digitalRead(4);

      //Steering:
      int encoder_err = absEncoder -> DEG(&outgoing_message.steering_angle);
      if(encoder_err == -1) {
        // that means we don't have the encoder, do something about it.
      }

      //Driving:
      //Distance calculation using incremental encoder data:
      unsigned int currPosition = (int) Enc -> read();
      newTheta = currPosition * 360.0 / 65536;

      if (newTheta != oldTheta) {
        if (newTheta - oldTheta > 300) {
          dist = dist + ((newTheta - 360 - oldTheta) / 360.0) * (0.2286 * Pi);
        }
        else if (newTheta - oldTheta < -300) {
          dist = dist + ((newTheta + 360 - oldTheta) / 360.0) * (0.2286 * Pi);
        }
        else {
          dist = dist + ((newTheta - oldTheta) / 360.0) * (0.2286 * Pi);
        }
        oldTheta = newTheta;

        outgoing_message.distance = dist;
      }

      memcpy(buffer + msg_size - sizeof(DriveSerialArduinoMsg), &outgoing_message, sizeof(DriveSerialArduinoMsg));
      Serial.write(buffer, msg_size);
      // Flush here?
    }

    char version_number;
    int err = read_n_bytes_from_serial(1, &version_number);
    
    if(err == -1) { 
      serial_state = CLEARING; 
      return; 
    }

    char incoming_serial_id_length;
    err = read_n_bytes_from_serial(1, &incoming_serial_id_length);

    if(err == -1) { 
      serial_state = CLEARING; 
      return; 
    }

    char serial_id_buffer[31];

    err = read_n_bytes_from_serial(incoming_serial_id_length, serial_id_buffer);

    if(err == -1) { 
      serial_state = CLEARING; 
      return; 
    }

    char data_buffer[255];
    err = read_n_bytes_from_serial(sizeof(DriveSerialComputerMsg), data_buffer);

    if(err == -1) { 
      serial_state = CLEARING; 
      return; 
    }

    memcpy(&incoming_msg, data_buffer, sizeof(DriveSerialComputerMsg));
    brushlessMotor1 -> PWM(incoming_msg.speed_motor1);
    brushlessMotor2 -> PWM(incoming_msg.speed_motor1);
    //delay(10);
  }
}
