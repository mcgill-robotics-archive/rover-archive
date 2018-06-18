// @TODO:
// Clean up

#include <Arduino.h>
#include <string.h>


#include "drive_serial_msgs.h"

#include <Servo.h>

#include "PID_v1.h"
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

short distance; // I WANT SPEED 1&2 FEEDBACK HERE
char fault; // boolean
char fuse; // boolean

//Define roverSide, topics and incremental encoder pinout according to board location on Rover:
DrivePosition location = FRONT_RIGHT; //Manually set according to board location on Rover
DriveSerialArduinoMsg outgoing_message;
DriveSerialComputerMsg incoming_msg = {};

double steering_angle  = 0.0;
double steering_target = 0.0;
double steering_PWM    = 0.0;

PID steering_pid = PID(&steering_angle, &steering_PWM, &steering_target, 7.0, 0.0, 0.0, DIRECT);

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

  steering_pid.SetMode(AUTOMATIC);
  steering_pid.SetOutputLimits(-100, 100);

  brushlessMotor1->PWM(0);
  brushlessMotor2->PWM(0);
  brushedMotor->PWM(0);

}

int read_n_bytes_from_serial(int n, char * buffer) {
  int counter = 0;
  unsigned long read_begin_time = millis();
  while(counter < n) {
    if(Serial.available() > 0) {
      buffer[counter++] = Serial.read();
    }

    if((millis() - read_begin_time) > 10000) return -1;
  }
  
  return 0;
}

void loop(){
  if(serial_state == CLEARING) {
    brushlessMotor1->PWM(0);
    brushlessMotor2->PWM(0);
    brushedMotor->PWM(0);
    while(Serial.available() > 0) {
      Serial.read();
      last_read_time = millis();
    }

    if(millis() - last_read_time > 3000) {
      serial_state = WAITING_FOR_HANDSHAKE;
    }
  } else if(serial_state == WAITING_FOR_HANDSHAKE) {
    brushlessMotor1->PWM(0);
    brushlessMotor2->PWM(0);
    brushedMotor->PWM(0);
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
      outgoing_message.fuse = steering_PWM;//digitalRead(4);

      //Steering:
      
      int encoder_err = absEncoder -> DEG(&steering_angle);
      if(encoder_err == -1) {
        // that means we don't have the encoder, do something about it.
      }
      outgoing_message.steering_angle = steering_angle;

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
      brushlessMotor1->PWM(0);
      brushlessMotor2->PWM(0);
      brushedMotor->PWM(0);
      return; 
    }

    char incoming_serial_id_length;
    err = read_n_bytes_from_serial(1, &incoming_serial_id_length);

    if(err == -1) { 
      serial_state = CLEARING; 
      brushlessMotor1->PWM(0);
      brushlessMotor2->PWM(0);
      brushedMotor->PWM(0);
      return; 
    }

    char serial_id_buffer[31];

    err = read_n_bytes_from_serial(incoming_serial_id_length, serial_id_buffer);

    if(err == -1) { 
      serial_state = CLEARING; 
      brushlessMotor1->PWM(0);
      brushlessMotor2->PWM(0);
      brushedMotor->PWM(0);
      return; 
    }

    char data_buffer[255];
    err = read_n_bytes_from_serial(sizeof(DriveSerialComputerMsg), data_buffer);

    if(err == -1) { 
      serial_state = CLEARING;
      brushlessMotor1->PWM(0);
      brushlessMotor2->PWM(0);
      brushedMotor->PWM(0);
      return; 
    }

    memcpy(&incoming_msg, data_buffer, sizeof(DriveSerialComputerMsg));
   
    brushlessMotor1->PWM(incoming_msg.speed_motor1);
    brushlessMotor2->PWM(incoming_msg.speed_motor2);
    
    steering_target = incoming_msg.steering_angle;
    
    while(steering_target < 0) {
      steering_target += 360;
    }

    if(steering_angle >= 180 && steering_target <= 180) {
      steering_target += 360;
    }

    if(steering_angle <= 180 && steering_target >= 180) {
      steering_target -= 360;
    }

    if(steering_target%360 > 90 && steering target%360 > 270) {
      steering_pid.Compute();
      brushedMotor->PWM(steering_PWM);
    } else {
      brushedMotor->PWM(0);
    }
    //delay(10);
  }
}
