
enum __attribute__((__packed__)) ArmPosition {
   FOREARM,
   BACKARM
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
    short angle_motor_A;
    short angle_motor_B;
    short angle_motor_C;
    short angle_motor_D;
    short speed_end_eff;
};

