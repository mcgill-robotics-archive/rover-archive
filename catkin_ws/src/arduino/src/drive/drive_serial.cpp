// @TODO:
// Create top level serial manager that assigns ports to specific decoders base on their id.
// Check match of message id.
// Handle multiple wheels.

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

#include "drive_serial_msgs.h"

// uncomment this to debug reads
#define SERIALPORTDEBUG 

#define SERIAL_VERSION 1
#define PORT "/dev/ttyACM0" // Temporary

enum SerialState {
    WAITING_FOR_HANDSHAKE, 
    CLEARING,
    FIRST_MESSAGE,
    RECEIVING
};

SerialState serial_state = WAITING_FOR_HANDSHAKE;

DriveSerialArduinoMsg last_received_msg;

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

    buf[i] = 0;  // null terminate the string
    return 0;
}

//
int serialport_flush(int fd)
{
    sleep(2); //required to make flush work, for some reason
    return tcflush(fd, TCIOFLUSH);
}

void close_and_reopen_port(int fd) {
    serialport_close(fd);

    usleep(10000000); // Give some time in case it's rebooting, might need adjustments

    do {
        printf("Reopen\n");
        fd = serialport_init(PORT, 9600);
    } while(fd == -1);

    serialport_flush(fd);
}

void receive_message(int fd) {
    char buf[4096] = {-1};
    int waht = serialport_read_n_bytes(fd, buf, sizeof(DriveSerialArduinoMsg)+7, 1000); // @TODO: Don't use hardcoded 7.

    if(waht == -2 || waht == -1) {
        close_and_reopen_port(fd);
        serial_state = WAITING_FOR_HANDSHAKE;
        return;            
    } else if (waht == 0) {
        DriveSerialArduinoMsg msg;

        memcpy(&msg, buf+7, sizeof(DriveSerialArduinoMsg));

        printf("Size: %ld, Version: %d, ID length: %d, ID:%c%c%c%c%c, Position: %d, Angle: %f, Distance %04x, Fault: %02x, Fuse: %02x\n", 
                sizeof(DriveSerialArduinoMsg), buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], msg.pos, msg.steering_angle, msg.distance, msg.fault, msg.fuse);

        last_received_msg = msg;

        serial_state = RECEIVING;
        return;
    }
}

void send_message(int fd) {
    const char * serial_id = "drive";
    int msg_size = 1 + 1 + strlen(serial_id) + sizeof(DriveSerialComputerMsg); // Constant
    char buffer[255];
    buffer[0] = SERIAL_VERSION;
    buffer[1] = (char) strlen(serial_id);
    memcpy(buffer + 2, serial_id, buffer[1]);

    DriveSerialComputerMsg outgoing_msg = {};
    outgoing_msg.pos = (DrivePosition) ((int)last_received_msg.pos + 1);

    //outgoing_msg.speed_motor1 = -100;

    memcpy(buffer + msg_size - sizeof(DriveSerialComputerMsg), &outgoing_msg, sizeof(DriveSerialComputerMsg));

    serialport_write(fd, (char *) buffer, msg_size); 
}

int main() {
    int fd = serialport_init(PORT, 9600);
    serialport_flush(fd);

    while(true) {
        if(serial_state == WAITING_FOR_HANDSHAKE) {
            char buf[4096] = {-1};
            int waht = serialport_read_until(fd, buf, '0', 1, 1000); // @TODO, to make this more robust, should be reading a large number of '0' in a row. Larger than max message size;
            if(waht == 0 && buf[0] == '0') {
                serialport_write(fd, "0", 1);
                serial_state = CLEARING;   
                continue;         
            } else {
                close_and_reopen_port(fd);
                continue;
            }
        } else if(serial_state == CLEARING) {
            char buf[4096] = {-1};
            int waht = serialport_read_until(fd, buf, 1, 1, 100); // clear '0's
            if(waht == -2 || waht == -1) {
                close_and_reopen_port(fd);
                serial_state = WAITING_FOR_HANDSHAKE;
                continue;            
            } else if (waht == 0 && buf[0] == 1) {
                serial_state = FIRST_MESSAGE;
                continue;
            }
        } else if(serial_state == FIRST_MESSAGE) {
            char buf[4096] = {-1};
            int waht = serialport_read_n_bytes(fd, buf+1, sizeof(DriveSerialArduinoMsg)-1+7, 1000); // @TODO: Don't use hardcoded 7.

            if(waht == -2 || waht == -1) {
                close_and_reopen_port(fd);
                serial_state = WAITING_FOR_HANDSHAKE;
                continue;            
            } else if (waht == 0) {
                DriveSerialArduinoMsg msg;

                memcpy(&msg, buf+7, sizeof(DriveSerialArduinoMsg));

                printf("Size: %ld, Version: %d, ID length: %d, ID:%c%c%c%c%c, Position: %d, Angle: %f, Distance %04x, Fault: %02x, Fuse: %02x\n", 
                        sizeof(DriveSerialArduinoMsg), buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], msg.pos, msg.steering_angle, msg.distance, msg.fault, msg.fuse);

                last_received_msg = msg;

                serial_state = RECEIVING;
                continue;
            }
            
        } else if(serial_state = RECEIVING) {
            send_message(fd);
            receive_message(fd);
            continue;
        }
    }
    return 0;
}
