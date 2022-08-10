#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H


#ifdef __cplusplus
#define SERIAL_API extern "C"
#else 
#define SERIAL_API
#endif


#include <stddef.h>


#ifdef _WIN32
//to do

#else
#include <termios.h>

#define SERIAL int
#define INVALID_SERIAL (-1)

#define BAUDRATE speed_t
#define BAUDRATE_LITERAL(n) B##n


#endif


// general baudrates

#define SERIAL_B300 BAUDRATE_LITERAL(300)
#define SERIAL_B600 BAUDRATE_LITERAL(600)
#define SERIAL_B1200 BAUDRATE_LITERAL(1200)
#define SERIAL_B2400 BAUDRATE_LITERAL(2400)
#define SERIAL_B9600 BAUDRATE_LITERAL(9600)
#define SERIAL_B19200 BAUDRATE_LITERAL(19200)
#define SERIAL_B38400 BAUDRATE_LITERAL(38400)
#define SERIAL_B57600 BAUDRATE_LITERAL(57600)
#define SERIAL_B230400 BAUDRATE_LITERAL(230400)
#define SERIAL_B460800 BAUDRATE_LITERAL(460800)
#define SERIAL_B921600 BAUDRATE_LITERAL(921600)


SERIAL_API int serial_num2baudrate(int num, BAUDRATE *br);

SERIAL_API  int serial_baudrate2num(BAUDRATE br, int * num);

SERIAL_API SERIAL serial_open(const char *dev, BAUDRATE br);

SERIAL_API int serial_read(SERIAL sp, void * buf, size_t sz);

SERIAL_API int serial_write(SERIAL sp, const void *data, size_t sz);

SERIAL_API int serial_clear(SERIAL  sp);

SERIAL_API int serial_close(SERIAL sp);



#endif