enum __attribute__((__packed__)) BoardPosition {
  FRONT_LEFT,
  FRONT_RIGHT,
  BACK_LEFT,
  BACK_RIGHT,
  FOREARM,
  BACKARM,
  POWER,
  SCIENCE
};

struct  __attribute__((packed, aligned(1))) DriveSerialArduinoMsg { // Rename to feedback
    DrivePosition pos;
    float steering_angle;
    short distance; // I WANT SPEED 1&2 FEEDBACK HERE
    char fault; // boolean
    char fuse; // boolean
};

struct __attribute__((packed, aligned(1))) DriveSerialComputerMsg { // Rename to command
    DrivePosition pos;
    float steering_angle;
    short speed_motor1;
    short speed_motor2;
    char reset; // boolean
};

struct  __attribute__((packed, aligned(1))) ArmSerialArduinoMsg {  //message going to the computer
    ArmPosition pos;
    float Angle_A;
    float Angle_B;
    float Angle_C;
    float Angle_D;
    float Current_A;
    float Current_B;
    float Current_C;
    float Current_D;
    float claw_position;
    char fault;               //make the char bit mapping for fuse and all faults
};

struct __attribute__((packed, aligned(1))) ArmSerialComputerMsg {   //message going to the arduino
    ArmPosition pos;
    float angle_motor_A;
    float angle_motor_B;
    float angle_motor_C;
    float angle_motor_D;
    float speed_end_eff;
};

struct  __attribute__((packed, aligned(1))) PowerSerialArduinoMsg {  //message going to the computer
    char fuse;        //each bit represents a fuse
    char share_state;
    float battery1_voltage;
    float battery2_voltage;
    float system_voltage;
};

struct __attribute__((packed, aligned(1))) PowerSerialComputerMsg {   //message going to the arduino
    char power_state;
    short angle_pitch;
    short angle_tilt;
};
