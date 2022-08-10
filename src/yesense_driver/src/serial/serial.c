#include "serial.h"
#include <stdio.h>



//if not
#ifndef _WIN32
//todo 
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#endif

struct BaudratePair{
    int number;
    BAUDRATE br;
};

#define BAUDRATE_PAIR(n) {n, BAUDRATE_LITERAL(n)}


static const struct BaudratePair BAUDRATEPAIRS[] = {
    BAUDRATE_PAIR(300),
    BAUDRATE_PAIR(600),
    BAUDRATE_PAIR(1200),
    BAUDRATE_PAIR(2400),
    BAUDRATE_PAIR(4800),
    BAUDRATE_PAIR(9600),
    BAUDRATE_PAIR(19200),
    BAUDRATE_PAIR(38400),
    BAUDRATE_PAIR(57600),
    BAUDRATE_PAIR(115200),
    BAUDRATE_PAIR(230400),
    BAUDRATE_PAIR(460800),
    BAUDRATE_PAIR(921600),
    BAUDRATE_PAIR(0)

};

//num convert to baudrate
int serial_num2baudrate(int num, BAUDRATE * br){
    const struct BaudratePair *pair = BAUDRATEPAIRS;

    for (; pair -> number; pair ++){
        if (num == pair -> number){
            *br = pair -> br;
            return 0;
        }
    }

    return -1;
}


// baudrate convert to num
int serial_baudrate2num(BAUDRATE br, int *num){
    const struct BaudratePair *pair = BAUDRATEPAIRS;

    for (; pair -> number; pair ++){
        if (br == pair ->br){
            * num =pair -> number;

            return 0;
        }
    }

    return -1;
}

//open serial 
SERIAL serial_open(const char* dev, BAUDRATE br){
    SERIAL serial = INVALID_SERIAL;

#ifdef _WIN32
    //todo

#else

    struct termios ios = {0};

    if ((serial = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK)) ==  -1){
        return INVALID_SERIAL;


    }

    tcgetattr(serial, &ios);

    ios.c_iflag = 0;
    ios.c_oflag = 0;
    ios.c_lflag = 0;
    ios.c_cc[VMIN] = 0;
    ios.c_cc[VTIME] = 0;
    cfsetispeed(&ios, br);
    cfsetospeed(&ios, br);

    //data bit 0
    ios.c_cflag &= ~CSIZE;
    ios.c_cflag |= CS8;
    //ignore modem controls

    ios.c_cflag |= (CLOCAL | CREAD);

    //no parity bit 奇偶校验
    ios.c_cflag &= ~(PARENB | PARODD);

    // stop bit 1
    ios.c_cflag &= ~CSTOPB;
    

    //no hardware flowcontrol, erpo  warnning but can ignore something todo with unart devie
    ios.c_cflag &= ~CRTSCTS;


    tcsetattr(serial, TCSANOW, &ios);
    tcflush(serial, TCIOFLUSH);

    return serial;




#endif
}


int serial_read(SERIAL sp, void *buf, size_t sz){
#ifdef _WIN32

//TODO
#else
    int rv = read(sp, buf, sz);
    if (rv == -1){
        if  (errno == EAGAIN|| errno == EWOULDBLOCK){
            return 0;
        }
        return -1;
    }else{
        return rv;
    }
#endif
}


int serial_write(SERIAL sp, const void *data,size_t sz){
#ifdef _WIN32
//todo
#else
    int rv = write(sp, data, sz);
    if (rv == -1){
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -1;
    }else return rv;
#endif
}

int serial_clear(SERIAL sp){
#ifdef _WIN32
//todo
#else 
    int rv = tcflush(sp, TCIOFLUSH);
    return rv;
#endif
}


int serial_close(SERIAL sp){
#ifdef _WIN32
//todo
#else
    int rv = close(sp);
    return rv;
#endif
}