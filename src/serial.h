#ifndef SERIAL_H
#define SERIAL_H

// Used for serial
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>
#include <errno.h>
#include <string>
#include <cstring>

#define SERIAL_TYPE_THS 0
#define SERIAL_TYPE_USB 1

#define SERIAL_READ  O_RDONLY
#define SERIAL_WRITE O_WRONLY
#define SERIAL_READWRITE O_RDWR

class Serial{

public:

    Serial( const char* device, int method = SERIAL_READ );


    void setup( int type, int baud = B115200 );

    bool read_char( uint8_t * c );
    int read_chars( uint8_t * buf, int maxlen );

    void write_chars( const uint8_t *c, int len );

    void write_string( const char * str );

private:

    int port_id;
    struct termios port_settings;

};

#endif // SERIAL_H
