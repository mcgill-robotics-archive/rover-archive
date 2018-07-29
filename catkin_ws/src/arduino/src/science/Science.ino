#include "Science.h" // include science library
#include "arduino_serial_msgs.h"
Science * science;

#define SERIAL_VERSION 1
// TODO: remove remove all the serial print stuff
// TODO: calibrate sensors 
// TODO: current monitoring
const char * serial_id = "science";

enum SerialFSM {
  CLEARING,
  WAITING_FOR_HANDSHAKE,
  TRANSMITTING
};

BoardPosition location = SCIENCE;
unsigned long last_read_time = 0;
SerialFSM serial_state = CLEARING;
ScienceSerialArduinoMsg outgoing_msg;
ScienceSerialComputerMsg incoming_msg = {};

void setup() {
    incoming_msg.pos = 0;
    science = new Science();
    Serial.begin(115200);
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
    //do something
    while(Serial.available() > 0) {
      Serial.read();
      last_read_time = millis();
    }

    if(millis() - last_read_time > 3000) {
      serial_state = WAITING_FOR_HANDSHAKE;
    }
  } else if(serial_state == WAITING_FOR_HANDSHAKE) {
    //do something
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
      int msg_size = 1+1+ strlen(serial_id) + sizeof(ScienceSerialArduinoMsg);
      char buffer[255];
      buffer[0] = SERIAL_VERSION;
      buffer[1] = (char) strlen(serial_id);
      memcpy(buffer+2, serial_id, buffer[1]);
  
      outgoing_msg.fault = 0x00;
      // limit switches
      outgoing_msg.fault += science->LimitSwitchCT()<<5;
      outgoing_msg.fault += science->LimitSwitchPT()<<4;
      outgoing_msg.fault += science->LimitSwitchCB()<<3;
      outgoing_msg.fault += science->LimitSwitchPT()<<2;
      if (science->controllerDrill->FLT()){outgoing_msg.fault += 0x02;}
      if (science->controllerDrill->FLT()){outgoing_msg.fault += 0x01;}
      if (science->LimitSwitchCT()){
        //stop drilling
        science->driveBDC(science->controllerDrill,0);
        science->driveBDC(science->controllerCRG,0);
      }
      if (science->LimitSwitchPT()){
        //stop probe
        science->servoTurnTo(science->servoProbe, 0);
      }
      if (science->LimitSwitchCB()){
        //stop drilling
        science->driveBDC(science->controllerDrill,0);
        science->driveBDC(science->controllerCRG,0);
      }
      if (science->LimitSwitchPB()){
        //stop probe
        science->servoTurnTo(science->servoProbe, 0);
      }
      
      outgoing_msg.currentC = science->currentMonitorDRG->readCurrent();
      outgoing_msg.currentD = science->currentMonitorDrill->readCurrent();
      
      memcpy(buffer + msg_size - sizeof(ArmSerialArduinoMsg), &outgoing_msg, sizeof(ArmSerialArduinoMsg));
      Serial.write(buffer, msg_size);
    }
    char version_number;
    int err = read_n_bytes_from_serial(1, &version_number);
    
    if(err == -1) { 
      serial_state = CLEARING; 
      science->controllerDrill->PWM(0);
      science->controllerCRG->PWM(0);
      return; 
    }

    char incoming_serial_id_length;
    err = read_n_bytes_from_serial(1, &incoming_serial_id_length);

    if(err == -1) { 
      serial_state = CLEARING;
      science->controllerDrill->PWM(0);
      science->controllerCRG->PWM(0); 
      return; 
    }

    char serial_id_buffer[31];

    err = read_n_bytes_from_serial(incoming_serial_id_length, serial_id_buffer);

    if(err == -1) { 
      serial_state = CLEARING; 
      science->controllerDrill->PWM(0);
      science->controllerCRG->PWM(0);
      return; 
    }

    char data_buffer[255];
    err = read_n_bytes_from_serial(sizeof(DriveSerialComputerMsg), data_buffer);

    if(err == -1) { 
      serial_state = CLEARING;
      science->controllerDrill->PWM(0);
      science->controllerCRG->PWM(0);
      return; 
    }

    memcpy(&incoming_msg, data_buffer, sizeof(DriveSerialComputerMsg));
    
    int ms_pwm = incoming_msg.ms_pwm;
    switch(incoming_msg.msid){
      case 0x00:
        //do something;
        break;
      case 0x01:
        science->servoProbe.write(ms_pwm);
        break;
      case 0x02:
        science->servoCmr.write(ms_pwm);
        break;
      case 0x03:
        science->servoLidRock.write(ms_pwm);
        break;
      case 0x04:
        science->servoLidSoil.write(ms_pwm);
        break;
      case 0x05:
        science->driveBDC(science->controllerDrill,ms_pwm);
        break;
      case 0x06:
        science->driveBDC(science->controllerDrill,ms_pwm);
        break;
      default:
        serial_state = CLEARING;
        break;
    } 
    
    outgoing_msg.data_type = incoming_msg.data_type;
    switch(incoming_msg.data_type){
      case 0x00:
        // do something
        break;
      case 0x01:
        outgoing_msg.data = science->windSpeed();
        break;
      case 0x02:
        outgoing_msg.data = science->uv();
        break;
      case 0x03:
        outgoing_msg.data = science->soilH();
        break;
      case 0x04:
        outgoing_msg.data = science->soilT();
        break;
      case 0x05:
        outgoing_msg.data = science->tempC();
        break;
      case 0x06:
        outgoing_msg.data = science->pascals();
        break;
      case 0x07:
        outgoing_msg.data = science->altm();
        break;
      case 0x08:
        outgoing_msg.data = science->tempC2();
        break;
      case 0x09:
        outgoing_msg.data = science->ah();
        break;
      case 0x0A:
        outgoing_msg.data = science->co2();
        break;
      case 0x0B:
        outgoing_msg.data = science->voc();
        break;
      case 0x0C:
        outgoing_msg.data = science->scale(science->scale_TA);
        break;
      case 0x0D:
        outgoing_msg.data = science->scale(science->scale_TB);
        break;
      case 0x0E:
        outgoing_msg.data = science->scale(science->scale_BA);
        break;
      case 0x0F:
        outgoing_msg.data = science->scale(science->scale_BA);
        break;
      default:
        serial_state = CLEARING;
        break;
    }
    
  }
    
}


//void loop() {
//  science->monitorCurrent();// not tested
//
//  /*drive motors*/
//  science->driveBDC(science->controllerDrill,0);
//  science->driveBDC(science->controllerCRG,0);
//
//  if (science->LimitSwitchPB()){
//    // do something
//    //servoTurnTo(SERVO_PROBE, 0);
//  }
//  if (science->LimitSwitchCB()){
//    // do something
//  }
//  if (science->LimitSwitchPT()){
//    //do something e.g.
//    science->servoTurnTo(science->servoProbe, 0);
//  }
//  if (science->LimitSwitchCT()){
//    //do something e.g.:
//    science->driveBDC(science->controllerDrill,0);
//    science->driveBDC(science->controllerCRG,0);
//  }
//  
//  //monitorMotor(MOTOR_DRILL);// WIP
//  //monitorMotor(MOTOR_CRG);
//
//  /*
//   * encoder calculation
//   * distance = DRILL_THREAD_DISTANCE*auger_angle_turned/360
//   * when distance >= AUGER_DEPTH -> stop auger
//   * (drill reaches its bottom)
//   */
//  science->auger_angle_turned = (science->encoderCRG->sum());/* or --> //science->auger_angle_turned += (science->encoderDrill->delta());*/
//  Serial.print("Auger has turned: " );
//  Serial.println(science->auger_angle_turned);
//  Serial.print("Auger has travelled: ");
//  Serial.println(science->auger_angle_turned*science->DRILL_THREAD_DISTANCE/360);
//  
//  if ( (( science->auger_angle_turned * science->DRILL_THREAD_DISTANCE / 360) >= (science->AUGER_DEPTH) ) || science->LimitSwitchCB()){
//    //stop auger;
//    science->driveBDC(science->controllerDrill,0);
//    science->driveBDC(science->controllerCRG,0);
//    //TODO: pull back auger;
//    
//  }
//
//    
//  int servoAngle = Serial.parseInt();//serial input: the angle to which the servo will rotate to.
//  science->servoTurnTo(science->servoProbe, servoAngle);
//  science->servoTurnTo(science->servoCmr, servoAngle);
//  science->servoTurnTo(science->servoLidRock ,servoAngle);
//  science->servoTurnTo(science->servoLidSoil, servoAngle);
//  /*
//   * Alternatively
//   */
//   science->servoProbe.write(servoAngle);
//   science->servoCmr.write(servoAngle);
//   science->servoLidRock.write(servoAngle);
//   science->servoLidSoil.write(servoAngle);
//}





