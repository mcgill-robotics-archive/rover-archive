// @TODO:
// Create top level serial manager that assigns ports to specific decoders base on their id.
// Check match of message id.
// Handle multiple wheels better (timing).

// MOSTLY TAKEN FROM https://github.com/todbot/arduino-serial/
// ACTUALLY USE THIS GUY's STUFF IF IT ENDS UP WORKING

#include <stdio.h>    // Standard input/output definitions 
#include <unistd.h>   // UNIX standard function definitions 
#include <fcntl.h>    // File control definitions 
#include <errno.h>    // Error number definitions 
#include <termios.h>  // POSIX terminal control definitions 
#include <string.h>   // String function definitions 
#include <sys/ioctl.h>
#include <stdint.h>
#include <sys/time.h>

#include "ros/ros.h"
#include "drive_control/WheelCommand.h"
#include "std_msgs/Float32.h"

#include "arduino_serial_msgs.h"

// uncomment this to debug reads
//#define SERIALPORTDEBUG 

#define SERIAL_VERSION 1
//Drive command
drive_control::WheelCommand command = drive_control::WheelCommand();

//Arm commands
float command_base_pitch 	= 0;
float command_base_yaw 		= 0;
float command_elbow_pitch 	= 0;
float command_elbow_roll 	= 0;
float command_wrist_pitch	= 0;
float command_wrist_roll 	= 0;
float command_speed_end_eff = 0;

//Power commands
float command_power_state = 0;
float command_angle_pitch = 0;
float command_angle_tilt  = 0;

enum SerialState {
    WAITING_FOR_HANDSHAKE, 
    CLEARING,
    FIRST_MESSAGE,
    RECEIVING,
    TIMEDOUT
};

struct Port {
    char * address;
    int fd;
    SerialState state = WAITING_FOR_HANDSHAKE;
    BoardPosition board;
    timeval previous_time;
    int timeout;
};

Port ports[4];

int num_boards = sizeof(ports)/sizeof(Port);

// takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;
    //printf("trying to open\n");
    //fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)  {
        perror("serialport_init: Unable to open port");
        return -1;
    }
    //printf("done openning %d\n", fd);
    //int iflags = TIOCM_DTR;
    //ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
    //ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR

    if (tcgetattr(fd, &toptions) < 0) {
        perror("serialport_init: Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud; // let you override switch below if needed
    switch(baud) {
    case 4800:   brate=B4800;   break;
    case 9600:   brate=B9600;   break;
#ifdef B14400
    case 14400:  brate=B14400;  break;
#endif
    case 19200:  brate=B19200;  break;
#ifdef B28800
    case 28800:  brate=B28800;  break;
#endif
    case 38400:  brate=B38400;  break;
    case 57600:  brate=B57600;  break;
    case 115200: brate=B115200; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;
    //toptions.c_cc[VTIME] = 20;
    
    tcsetattr(fd, TCSANOW, &toptions);
    if( tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}

//
int serialport_close( int fd )
{
    return close( fd );
}

//
int serialport_write(int fd, const char* str, int length)
{
    int n = write(fd, str, length);

//    int i = 0;
//    while(i<n) {
//        printf("%02x\n", str[i++]);
//    
//    }

    if( n!=length ) {
        perror("serialport_write: couldn't write whole string\n");
        return -1;
    }
    return 0;
}

//
int serialport_read_n_bytes(int fd, char * buf, int n,  int timeout) {
    char b[1];  // read expects an array, so we give it a 1-byte array
    int i=0;
    while(i < n && timeout>0) { 
        int n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
            usleep( 1 * 1000 );  // wait 1 msec try again
            timeout--;
            if( timeout==0 ) return -2;
            continue;
        }
#ifdef SERIALPORTDEBUG  
        printf("serialport_read_until: i=%d, n=%d b='%02x'\n",i,n,b[0]); // debug
#endif
        buf[i] = b[0]; 
        i++;
    } 

    // Don't do that actually. buf[i] = 0;  // null terminate the string
    return 0;
}

//
int serialport_read_until(int fd, char* buf, char until, int buf_max, int timeout)
{
    char b[1];  // read expects an array, so we give it a 1-byte array
    int i=0;
    do { 
        int n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
            usleep( 1 * 1000 );  // wait 1 msec try again
            timeout--;
            if( timeout==0 ) return -2;
            continue;
        }
#ifdef SERIALPORTDEBUG  
        printf("serialport_read_until: i=%d, n=%d b='%02x'\n",i,n,b[0]); // debug
#endif
        buf[i] = b[0]; 
        i++;
    } while( b[0] != until && i < buf_max && timeout>0 );

    buf[0] = b[0];

    return 0;
}

//
int serialport_flush(int fd)
{
    sleep(2); //required to make flush work, for some reason
    return tcflush(fd, TCIOFLUSH);
}

int reopen_port(Port * port) {
    printf("Reopen %s\n", port->address);
    return serialport_init(port->address, 9600);
}

void drive_receive_message(Port * port) {
    char buf[4096] = {-1};
    int waht = serialport_read_n_bytes(port->fd, buf, sizeof(DriveSerialArduinoMsg)+7, 1000); // @TODO: Don't use hardcoded 7.

    if(waht == -2 || waht == -1) {
        port->timeout = 10000;
        serialport_close(port->fd);
        port->state = TIMEDOUT;
        return;            
    } else if (waht == 0) {
        DriveSerialArduinoMsg msg;

        memcpy(&msg, buf+7, sizeof(DriveSerialArduinoMsg));

        printf("Port: %s, Version: %d, ID length: %d, ID:%c%c%c%c%c, Position: %d, Angle: %f, Distance %04x, Fault: %02x, Fuse: %02x\n", 
                port->address, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], msg.pos, msg.steering_angle, msg.distance, msg.fault, msg.fuse);

        port->board = msg.pos;

        port->state = RECEIVING;
        return;
    }
}

void drive_send_message(Port* port) {
    const char * serial_id = "drive";
    int msg_size = 1 + 1 + strlen(serial_id) + sizeof(DriveSerialComputerMsg); // Constant
    char buffer[255];
    buffer[0] = SERIAL_VERSION;
    buffer[1] = (char) strlen(serial_id);
    memcpy(buffer + 2, serial_id, buffer[1]);

    DriveSerialComputerMsg outgoing_msg = {};

    BoardPosition position = port->board;

    outgoing_msg.pos = position;

    if(position == FRONT_LEFT) {

        outgoing_msg.speed_motor1   = command.flv;
        outgoing_msg.speed_motor2   = command.mlv;
        outgoing_msg.steering_angle = command.flsa;

    } else if(position == FRONT_RIGHT) {

        outgoing_msg.speed_motor1   = command.frv;
        outgoing_msg.speed_motor2   = command.mrv;
        outgoing_msg.steering_angle = command.frsa;

    } else if(position == BACK_LEFT) {

        outgoing_msg.speed_motor1   = command.blv;
        outgoing_msg.steering_angle = command.blsa;

    } else if(position == BACK_RIGHT) {

        outgoing_msg.speed_motor1   = command.brv;
        outgoing_msg.steering_angle = command.brsa;

    }

    memcpy(buffer + msg_size - sizeof(DriveSerialComputerMsg), &outgoing_msg, sizeof(DriveSerialComputerMsg));

    serialport_write(port->fd, (char *) buffer, msg_size);
}

void arm_receive_message(Port * port) {
    char buf[4096] = {-1};
    int waht = serialport_read_n_bytes(port->fd, buf, sizeof(ArmSerialArduinoMsg)+7, 1000); // @TODO: Don't use hardcoded 7.

    if(waht == -2 || waht == -1) {
        port->timeout = 10000;
        serialport_close(port->fd);
        port->state = TIMEDOUT;
        return;
    } else if (waht == 0) {
        ArmSerialArduinoMsg msg;

        memcpy(&msg, buf+7, sizeof(ArmSerialArduinoMsg));

        printf("Port: %s, Version: %d, ID length: %d, ID:%c%c%c%c%c, Angle_A: %f, Angle_B: %f, Angle_C: %f, Angle_D: %f, Current_A: %f, Current_B: %f, Current_C: %f, Current_D: %f, Claw position: %f, Fault: %02x\n",
                port->address, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], msg.Angle_A, msg.Angle_B, msg.Angle_C, msg.Angle_D, msg.Current_A, msg.Current_B, msg.Current_C, msg.Current_D, msg.claw_position, msg.fault);

        port->board = msg.pos;

        port->state = RECEIVING;
        return;
    }
}

void arm_send_message(Port* port) {
    const char * serial_id = "armer";
    int msg_size = 1 + 1 + strlen(serial_id) + sizeof(ArmSerialComputerMsg); // Constant
    char buffer[255];
    buffer[0] = SERIAL_VERSION;
    buffer[1] = (char) strlen(serial_id);
    memcpy(buffer + 2, serial_id, buffer[1]);

    ArmSerialComputerMsg outgoing_msg = {};

    BoardPosition position = port->board;

    outgoing_msg.pos = position;

    if(position == FOREARM) {

        outgoing_msg.angle_motor_A  = command_wrist_pitch;
        outgoing_msg.angle_motor_B  = command_wrist_roll;
        outgoing_msg.speed_end_eff  = command_speed_end_eff;               //TO DO

    } else if(position == BACKARM) {
    	outgoing_msg.angle_motor_A  = command_elbow_pitch;
    	outgoing_msg.angle_motor_B  = command_elbow_roll;
    	outgoing_msg.angle_motor_C  = command_base_pitch;
    	outgoing_msg.angle_motor_D  = command_base_yaw;

    }

    memcpy(buffer + msg_size - sizeof(ArmSerialComputerMsg), &outgoing_msg, sizeof(ArmSerialComputerMsg));

    serialport_write(port->fd, (char *) buffer, msg_size);
}

void power_receive_message(Port * port) {
    char buf[4096] = {-1};
    int waht = serialport_read_n_bytes(port->fd, buf, sizeof(PowerSerialArduinoMsg)+7, 1000); // @TODO: Don't use hardcoded 7.

    if(waht == -2 || waht == -1) {
        port->timeout = 10000;
        serialport_close(port->fd);
        port->state = TIMEDOUT;
        return;
    } else if (waht == 0) {
        PowerSerialArduinoMsg msg;

        memcpy(&msg, buf+7, sizeof(PowerSerialArduinoMsg));

        printf("Port: %s, Version: %d, ID length: %d, ID:%c%c%c%c%c, Share_State: %c, Battery 1(V): %f, Battery 2(V): %f, System(V): %f, Fuse: %02x\n",
                port->address, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], msg.share_state, msg.battery1_voltage, msg.battery2_voltage, msg.system_voltage, msg.fuse);

        port->board = msg.pos;

        port->state = RECEIVING;
        return;
    }
}

void power_send_message(Port* port) {
    const char * serial_id = "power";
    int msg_size = 1 + 1 + strlen(serial_id) + sizeof(PowerSerialComputerMsg); // Constant
    char buffer[255];
    buffer[0] = SERIAL_VERSION;
    buffer[1] = (char) strlen(serial_id);
    memcpy(buffer + 2, serial_id, buffer[1]);

    PowerSerialComputerMsg outgoing_msg = {};

    outgoing_msg.power_state = command_power_state;
    outgoing_msg.angle_pitch = command_angle_pitch;
    outgoing_msg.angle_tilt  = command_angle_tilt;

    memcpy(buffer + msg_size - sizeof(PowerSerialComputerMsg), &outgoing_msg, sizeof(PowerSerialComputerMsg));

    serialport_write(port->fd, (char *) buffer, msg_size);
}

void wheel_command_cb(const drive_control::WheelCommand::ConstPtr& msg)
{
    command = *msg;
}

void arm_base_pitch_command_cb(const std_msgs::Float32::ConstPtr& msg)
{
    command_base_pitch = msg->data;
}
void arm_base_yaw_command_cb(const std_msgs::Float32::ConstPtr& msg)
{
    command_base_yaw = msg->data;
}
void arm_elbow_pitch_command_cb(const std_msgs::Float32::ConstPtr& msg)
{
    command_elbow_pitch = msg->data;
}
void arm_elbow_roll_command_cb(const std_msgs::Float32::ConstPtr& msg)
{
    command_elbow_roll = msg->data;
}
void arm_wrist_pitch_command_cb(const std_msgs::Float32::ConstPtr& msg)
{
    command_wrist_pitch = msg->data;
}
void arm_wrist_roll_command_cb(const std_msgs::Float32::ConstPtr& msg)
{
    command_wrist_roll = msg->data;
}
void arm_speed_end_eff_command_cb(const std_msgs::Float32::ConstPtr& msg)
{
    command_speed_end_eff = msg->data;
}
void power_pitch_command_cb(const std_msgs::Float32::ConstPtr& msg)
{
    command_angle_pitch = msg->data;
}
void power_tilt_command_cb(const std_msgs::Float32::ConstPtr& msg)
{
    command_angle_tilt = msg->data;
}

int main(int argc, char *argv []) {

    ros::init(argc, argv, "arduino_serial_controller");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/wheel_command", 1000, wheel_command_cb);
    ros::Subscriber base_pitch_sub = n.subscribe("/arm/base_pitch_position_controller/command", 1000, arm_base_pitch_command_cb);
    ros::Subscriber base_yaw_sub = n.subscribe("/arm/base_yaw_position_controller/command", 1000, arm_base_yaw_command_cb);
    ros::Subscriber elbow_pitch_sub = n.subscribe("/arm/elbow_pitch_position_controller/command", 1000, arm_elbow_pitch_command_cb);
    ros::Subscriber elbow_roll_sub = n.subscribe("/arm/elbow_roll_position_controller/command", 1000, arm_elbow_roll_command_cb);
    ros::Subscriber wrist_pitch_sub = n.subscribe("/arm/wrist_pitch_position_controller/command", 1000, arm_wrist_pitch_command_cb);
    ros::Subscriber wrist_roll_sub = n.subscribe("/arm/wrist_roll_position_controller/command", 1000, arm_wrist_roll_command_cb);
    //TODO : add subscribers for power and science
    //TODO : add publishers for everyone

    ports[0].address = "/dev/ttyACM0";
    ports[0].state = WAITING_FOR_HANDSHAKE;

    ports[1].address = "/dev/ttyACM1";
    ports[1].state = WAITING_FOR_HANDSHAKE;

    ports[2].address = "/dev/ttyACM2";
    ports[2].state = WAITING_FOR_HANDSHAKE;

    ports[3].address = "/dev/ttyACM3";
    ports[3].state = WAITING_FOR_HANDSHAKE;

//    ports[4].address = "/dev/ttyACM4";
//    ports[4].state = WAITING_FOR_HANDSHAKE;

    for(int i = 0; i < num_boards; i++) {
        ports[i].fd = serialport_init(ports[i].address, 9600);
        serialport_flush(ports[i].fd);
    }

    while(ros::ok()) {

        ros::spinOnce();
        for(int i = 0; i < num_boards; i++) {

            Port * port = &ports[i];

            printf("\n\n\nBoard %s, State %d\n", port->address, port->state);

            if(port->state == WAITING_FOR_HANDSHAKE) {
                
                char buf[4096] = {-1};
                int waht = serialport_read_n_bytes(port->fd, buf, 512, 1000);

                bool ready = true;

                for(int byte_count = 0; byte_count < 512; byte_count++) {
                    if(buf[byte_count] != '0') {
                        ready = false;                        
                    }                
                }
                printf("%x\n", buf[0]);
                if(waht == 0 && ready) {
                    serialport_write(port->fd, "0", 1);
                    port->state = CLEARING;   
                    //continue;         
                } else {
                    port->timeout = 10000;
                    serialport_close(port->fd);
                    port->state = TIMEDOUT;
                    //continue;
                }

                
            } else if(port->state == CLEARING) {
                char buf[4096] = {-1};
                int waht = serialport_read_until(port->fd, buf, SERIAL_VERSION, 4096, 10000); // clear '0's
                if(waht == -2 || waht == -1) {
                    port->timeout = 10000;
                    serialport_close(port->fd);
                    port->state = TIMEDOUT;
                    //continue;            
                } else if (waht == 0 && buf[0] == 1) {
                    port->state = FIRST_MESSAGE;
                    //continue;
                }

            } else if(port->state == FIRST_MESSAGE) {
            	char buf[4096] = {-1};
            	if(port->board == FRONT_LEFT || port->board == FRONT_RIGHT || port->board == BACK_LEFT || port->board == BACK_RIGHT){
            		int waht = serialport_read_n_bytes(port->fd, buf+1, sizeof(DriveSerialArduinoMsg)-1+7, 1000); // @TODO: Don't use hardcoded 7
            		if(waht == -2 || waht == -1) {
            			port->timeout = 10000;
            		    serialport_close(port->fd);
            		    port->state = TIMEDOUT;
            		    //continue;
            		} else if (waht == 0) {
            			DriveSerialArduinoMsg msg;

            		    memcpy(&msg, buf+7, sizeof(DriveSerialArduinoMsg));

            		    printf("Port: %s, Version: %d, ID length: %d, ID:%c%c%c%c%c, Position: %d, Angle: %f, Distance %04x, Fault: %02x, Fuse: %02x\n",
            		            port->address, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], msg.pos, msg.steering_angle, msg.distance, msg.fault, msg.fuse);
            		    port->board = msg.pos;
     		            port->state = RECEIVING;
            		    //continue;
            		}
            	}
            	else if(port->board == FOREARM || port->board == BACKARM){
            		int waht = serialport_read_n_bytes(port->fd, buf+1, sizeof(ArmSerialArduinoMsg)-1+7, 1000); // @TODO: Don't use hardcoded 7.

            		if(waht == -2 || waht == -1) {
            			port->timeout = 10000;
            		    serialport_close(port->fd);
            		    port->state = TIMEDOUT;
            		    //continue;
            		} else if (waht == 0) {
            			ArmSerialArduinoMsg msg;

            		    memcpy(&msg, buf+7, sizeof(ArmSerialArduinoMsg));

            		    printf("Port: %s, Version: %d, ID length: %d, ID:%c%c%c%c%c, Angle_A: %f, Angle_B: %f, Angle_C: %f, Angle_D: %f, Current_A: %f, Current_B: %f, Current_C: %f, Current_D: %f, Claw position: %f, Fault: %02x\n",
            		            port->address, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], msg.Angle_A, msg.Angle_B, msg.Angle_C, msg.Angle_D, msg.Current_A, msg.Current_B, msg.Current_C, msg.Current_D, msg.claw_position, msg.fault);
            		    port->board = msg.pos;
            		    port->state = RECEIVING;
            		}
            	}
            	else if(port->board == POWER){
            		int waht = serialport_read_n_bytes(port->fd, buf, sizeof(PowerSerialArduinoMsg)+7, 1000); // @TODO: Don't use hardcoded 7.

            		if(waht == -2 || waht == -1) {
            			port->timeout = 10000;
            		    serialport_close(port->fd);
            		    port->state = TIMEDOUT;

            		} else if (waht == 0) {
            		    PowerSerialArduinoMsg msg;

            		    memcpy(&msg, buf+7, sizeof(PowerSerialArduinoMsg));

            		    printf("Port: %s, Version: %d, ID length: %d, ID:%c%c%c%c%c, Share_State: %c, Battery 1(V): %f, Battery 2(V): %f, System(V): %f, Fuse: %02x\n",
            		    		port->address, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], msg.share_state, msg.battery1_voltage, msg.battery2_voltage, msg.system_voltage, msg.fuse);
            		    port->board = msg.pos;
            		    port->state = RECEIVING;
            	}
            	else if(port->board == SCIENCE){

            	}
                
            } else if(port->state == RECEIVING) {
            	if(port->board == FRONT_LEFT || port->board == FRONT_RIGHT || port->board == BACK_LEFT || port->board == BACK_RIGHT){
            		drive_send_message(port);
            		drive_receive_message(port);
            	}
            	else if(port->board == FOREARM || port->board == BACKARM){
            		arm_send_message(port);
            		arm_receive_message(port);
            	}
            	else if(port->board == POWER){
            		power_send_message(port);
            		power_receive_message(port);
            	}
            	else if(port->board == SCIENCE){

            	}
                //continue;
            } else if(port->state == TIMEDOUT) {
                if(port->timeout > 0) {

                    if(port->previous_time.tv_sec == 0 && port->previous_time.tv_usec == 0) { gettimeofday(&(port->previous_time), NULL); } // Init previous time for the first time.

                    timeval current_time;
                    gettimeofday(&current_time, NULL);
                    int elapsed_time = 1000*(current_time.tv_sec - port->previous_time.tv_sec) + (current_time.tv_usec - port->previous_time.tv_usec) / 1000;

                    port->timeout -= elapsed_time;

                    port->previous_time = current_time;
                } else {
                    port->fd = reopen_port(port);
                    if(port->fd != 1) {
                        serialport_flush(port->fd);
                        port->previous_time.tv_sec  = 0;
                        port->previous_time.tv_usec = 0;
                        port->state = WAITING_FOR_HANDSHAKE;
                    }
                }       
            }
        }
    }
    return 0;
}
