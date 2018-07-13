struct  __attribute__((packed, aligned(1))) PowerSerialArduinoMsg {  //message going to the computer
    char fuse;        //each bit represents a fuse
    char share_state;
};

struct __attribute__((packed, aligned(1))) PowerSerialComputerMsg {   //message going to the arduino
    char power_state;
    short angle_pitch;
    short angle_tilt;
};
