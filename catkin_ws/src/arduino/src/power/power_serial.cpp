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

#include "power_serial_msgs.h"

// uncomment this to debug reads
//#define SERIALPORTDEBUG 

#define SERIAL_VERSION 1

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
    PowerSerialArduinoMsg last_received_msg;
    timeval previous_time;
    int timeout;
};

Port ports;

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

void receive_message(Port * port) {
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

        printf("Port: %s, Version: %d, ID length: %d, ID:%c%c%c%c%c, Share_State: %c, Fuse: %02x\n",
                port->address, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], msg.share_state, msg.fuse);

        port->last_received_msg = msg;

        port->state = RECEIVING;
        return;
    }
}

void send_message(Port* port) {
    const char * serial_id = "power";
    int msg_size = 1 + 1 + strlen(serial_id) + sizeof(PowerSerialComputerMsg); // Constant
    char buffer[255];
    buffer[0] = SERIAL_VERSION;
    buffer[1] = (char) strlen(serial_id);
    memcpy(buffer + 2, serial_id, buffer[1]);

    PowerSerialComputerMsg outgoing_msg = {};

    outgoing_msg.power_state = command.power_state;
    outgoing_msg.angle_pitch = command.angle_pitch;
    outgoing_msg.angle_tilt  = command.angle_tilt;

    memcpy(buffer + msg_size - sizeof(PowerSerialComputerMsg), &outgoing_msg, sizeof(PowerSerialComputerMsg));

    serialport_write(port->fd, (char *) buffer, msg_size);
}

void power_command_cb(const rover_common::PowerCommand::ConstPtr& msg){
	command = *msg;
}

int main(int argc, char *argv []) {

    ros::init(argc, argv, "power_serial_controller");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/power_command", 1000, power_command_cb);

    ports.address = "/dev/ttyACM4";						//verify usb
    ports.state = WAITING_FOR_HANDSHAKE;

    ports.fd = serialport_init(ports.address, 9600);
    serialport_flush(ports.fd);

    while(ros::ok()) {

        ros::spinOnce();
        Port * port = &ports;

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
                int waht = serialport_read_n_bytes(port->fd, buf+1, sizeof(PowerSerialArduinoMsg)-1+7, 1000); // @TODO: Don't use hardcoded 7.

                if(waht == -2 || waht == -1) {
                    port->timeout = 10000;
                    serialport_close(port->fd);
                    port->state = TIMEDOUT;
                    //continue;            
                } else if (waht == 0) {
                    PowerSerialArduinoMsg msg;

                    memcpy(&msg, buf+7, sizeof(PowerSerialArduinoMsg));
                    
                    printf("Port: %s, Version: %d, ID length: %d, ID:%c%c%c%c%c, Share State: %c, Fuse: %02x\n",
                            port->address, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], msg.share_state, msg.fuse);
                    port->last_received_msg = msg;

                    port->state = RECEIVING;
                    //continue;
                }
                
            } else if(port->state == RECEIVING) {
                send_message(port);
                receive_message(port);
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
    return 0;
}
