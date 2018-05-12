//Drive System, Electrical Division, Mars Rover Team of McGill Robotics
//Raymond So, Amanda Bianco, Lin Miao and Wendy Li
//Winter 2018

//Drive and steering code for each board (4) of the Mars Rover's drive system. It includes:
//      -Publisher for steering functionality per board: absolute encoder angle
//      -Subscriber for steering  functionality per board: brushed motor
//      -Publisher for driving functionality per board: incremental encoder distance
//      -Subscribers for driving functionality per board: brushless motors
//      -Fault detection publisher for each brushed driver for steering
//      -Fuse detection publisher per board
//      -Common reset subscriber for all brushed drivers for steering


#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

#include "BLDC_AfroESC/BLDC_AfroESC.h"
#include "BDC_DRV8842/BDC_DRV8842.h"
#include "AMT203-V_ABS/AMT203-V_ABS.h"

#include <Servo.h>
#include <Encoder/Encoder.h>


ros::NodeHandle driveSystem;


BDC * brushedMotor;
AMT_ABS * absEncoder;
BLDC * brushlessMotor1;
BLDC * brushlessMotor2;


//Variables for distance calculation using incremental encoder data:
int oldTheta = 0;
int newTheta = 0;
double dist = 0.0;
const float Pi = 3.14159;


//Specify board location:
enum boardLocation {
  FRONT_LEFT,
  FRONT_RIGHT,
  BACK_LEFT,
  BACK_RIGHT
};

boardLocation location = FRONT_LEFT; //Manually set according to board location on Rover


//Publishers and subscribers for steering:
const char * steering_topic1;
const char * steering_topic2;

std_msgs::Int16 absEncoder_angle; 
ros::Publisher * steeringEncoder;

void steering_messageCb( const std_msgs::Int16& steering_msg) {
  brushedMotor -> PWM(steering_msg.data);
}
ros::Subscriber<std_msgs::Int16> * steeringMotor;


//Publishers and subscribers for driving:
Side roverSide;
const char * drive_topic1;
const char * drive_topic2;
const char * drive_topic3;
Encoder * Enc;

std_msgs::Int16 incEncoder_distance;
ros::Publisher * driveEncoder;

void driveMotor1_messageCb( const std_msgs::Int16& driveMotor1_msg) {
  brushlessMotor1 -> PWM(driveMotor1_msg.data);
}
ros::Subscriber<std_msgs::Int16> * driveMotor1;

void driveMotor2_messageCb( const std_msgs::Int16& driveMotor2_msg) {
  brushlessMotor2 -> PWM(driveMotor2_msg.data);
}
ros::Subscriber<std_msgs::Int16> * driveMotor2;


//Publisher for fault detection:
const char * fault_topic;

std_msgs::Bool driveFaultStatus;
ros::Publisher * steeringFault;


//Publisher for fuse detection:
const char * fuse_topic;

std_msgs::Bool driveFuseStatus;
ros::Publisher * driveFuse;


//Subscriber for reset:

void steeringReset_messageCb( const std_msgs::Bool& steeringReset_msg) {
  brushedMotor -> RST();
}
ros::Subscriber<std_msgs::Bool> * steeringReset;


void setup() {

  //Define roverSide, topics and incremental encoder pinout according to board location on Rover:

  if (location == FRONT_RIGHT) {

    roverSide = RIGHT;

    steering_topic1 = "FR_absAngle";
    steering_topic2 = "FR_steeringPWM";
    drive_topic1 = "R_drivePWM";
    drive_topic2 = "MR_drivePWM";
    drive_topic3 = "FR_distance";
    fuse_topic = "FR_fuse";
    fault_topic = "FR_fault";

    Enc = new Encoder(3, 2);  /* Rignt wheel: (B, A),  Left wheel: (A, B)*/
  }

  else if (location == FRONT_LEFT) {

    roverSide = LEFT;

    steering_topic1 = "FL_absAngle";
    steering_topic2 = "FL_steeringPWM";
    drive_topic1 = "L_drivePWM";
    drive_topic2 = "ML_drivePWM";
    drive_topic3 = "FL_distance";
    fuse_topic = "FL_fuse";
    fault_topic = "FL_fault";

    Enc = new Encoder(3, 2);  /* Rignt wheel: (B, A),  Left wheel: (A, B)*/
  }

  else if (location == BACK_RIGHT) {

    roverSide = RIGHT;

    steering_topic1 = "BR_absAngle";
    steering_topic2 = "BR_steeringPWM";
    drive_topic1 = "R_drivePWM";
    drive_topic2 = "MR_drivePWM";
    drive_topic3 = "BR_distance";
    fuse_topic = "BR_fuse";
    fault_topic = "BR_fault";

    Enc = new Encoder(3, 2);  /* Rignt wheel: (B, A),  Left wheel: (A, B)*/
  }

  else if (location == BACK_LEFT) {

    roverSide = LEFT;

    steering_topic1 = "BL_absAngle";
    steering_topic2 = "BL_steeringPWM";
    drive_topic1 = "L_drivePWM";
    drive_topic2 = "ML_drivePWM";
    drive_topic3 = "BL_distance";
    fuse_topic = "BL_fuse";
    fault_topic = "BL_fault";

    Enc = new Encoder(2, 3);  /* Rignt wheel: (B, A),  Left wheel: (A, B)*/
  }

  driveSystem.initNode();

  //Steering:
  steeringEncoder = new ros::Publisher(steering_topic1, &absEncoder_angle);
  driveSystem.advertise(*steeringEncoder);

  steeringMotor = new ros::Subscriber<std_msgs::Int16>(steering_topic2, &steering_messageCb);
  driveSystem.subscribe(*steeringMotor);

  brushedMotor = new BDC(11, 13, 7, 8);
  absEncoder = new AMT_ABS(SS);


  //Driving:
  driveEncoder = new ros::Publisher(drive_topic3, &incEncoder_distance);
  driveSystem.advertise(*driveEncoder);

  driveMotor1 =  new ros::Subscriber<std_msgs::Int16>(drive_topic1, &driveMotor1_messageCb);
  driveSystem.subscribe(*driveMotor1);

  driveMotor2 = new ros::Subscriber<std_msgs::Int16>(drive_topic2, &driveMotor2_messageCb);
  driveSystem.subscribe(*driveMotor2);

  brushlessMotor1 = new BLDC(6, roverSide);
  brushlessMotor2 = new BLDC(5, roverSide);


  //Fault Detection:
  steeringFault = new ros::Publisher(fault_topic, &driveFaultStatus);
  driveSystem.advertise(*steeringFault);
  
  pinMode(4, OUTPUT);


  //Fuse Detection:
  driveFuse = new ros::Publisher(fuse_topic, &driveFuseStatus);
  driveSystem.advertise(*driveFuse);

  //Reset:
  steeringReset = new ros::Subscriber<std_msgs::Bool>("steeringReset", &steeringReset_messageCb);
  driveSystem.subscribe(*steeringReset);
}



void loop() {

  //Fault Detection:
  driveFaultStatus.data = brushedMotor -> FLT();
  if (driveFaultStatus.data == HIGH) {
    steeringFault -> publish(&driveFaultStatus);
  }


  //Fuse Detection:
  driveFuseStatus.data = digitalRead(4);
  if (driveFuseStatus.data == HIGH) { 
    driveFuse -> publish(&driveFuseStatus);
  }


  //Steering:
  absEncoder_angle.data = absEncoder -> DEG();
  steeringEncoder -> publish(&absEncoder_angle);


  //Driving:
  //Distance calculation using incremental encoder data:
  unsigned int currPosition = Enc -> read();
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

    incEncoder_distance.data = dist;
    driveEncoder -> publish(&incEncoder_distance);
  }


  driveSystem.spinOnce();
  delay(1);

}







