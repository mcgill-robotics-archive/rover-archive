

enum __attribute__((__packed__)) DrivePosition {
  FRONT_LEFT,
  FRONT_RIGHT,
  BACK_LEFT,
  BACK_RIGHT
};

struct  __attribute__((packed, aligned(1))) DriveSerialArduinoMsg {
    DrivePosition pos;
    float steering_angle;
    short distance; // I WANT SPEED 1&2 FEEDBACK HERE
    char fault; // boolean
    char fuse; // boolean
};

struct __attribute__((packed, aligned(1))) DriveSerialComputerMsg {
    DrivePosition pos;
    short speed_motor1;
    short speed_motor2;
    char reset; // boolean
};
