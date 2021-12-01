#include "serial.h"

Serial::Serial( const char* device, int method )
{
    // "/dev/ttyTHS1"
    port_id = open( device, method );

    if (port_id < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    // Flush buffer
    tcflush(port_id, TCIFLUSH);
}


void Serial::setup( int type, int baud ){
    // Get serial port settings

    if( tcgetattr(port_id, &port_settings) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    // Configure serial port
    switch (type) {
        case SERIAL_TYPE_THS:
            port_settings.c_cflag &= ~PARENB;                            // Disables the Parity Enable bit(PARENB),So No Parity
            port_settings.c_cflag &= ~CSTOPB;                            // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
            port_settings.c_cflag &= ~CSIZE;                             // Clears the mask for setting the data size
            port_settings.c_cflag |=  CS8;                               // Set the data bits = 8
            port_settings.c_cflag &= ~CRTSCTS;                           // No Hardware flow Control
            port_settings.c_cflag |=  CREAD | CLOCAL;                    // Enable receiver,Ignore Modem Control lines
            port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);            // Disable XON/XOFF flow control both input & output
            port_settings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);    // Non Cannonical mode
            port_settings.c_oflag &= ~OPOST;                             // No Output Processing
            port_settings.c_lflag = 0;                                   // Enable raw input instead of canonical,
            port_settings.c_cc[VMIN]  = 0;                               // Read at least 1 character
            port_settings.c_cc[VTIME] = 0;                               // Wait indefinetly
            break;

        case SERIAL_TYPE_USB:
            port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);            // Disable XON/XOFF flow control both input & output
            port_settings.c_lflag = 0;                                   // Enable raw input instead of canonical,
            port_settings.c_cc[VMIN]  = 0;                               // Read at least 1 character
            port_settings.c_cc[VTIME] = 0;                               // Wait indefinetly
            break;

        default:
            printf("Unvalid configuration type");
            break;
    }

    // Set baudrate
    cfsetospeed(&port_settings, baud );

    // Save port_settings
    if (tcsetattr(port_id, TCSANOW, &port_settings) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
}


bool Serial::read_char( uint8_t * c )
{
    if( read( port_id, c, 1) != -1 ){
        return true;
    }
    return false;
}

int Serial::read_chars( uint8_t * buf, int maxlen )
{
    return read( port_id, buf, maxlen );
}

void Serial::write_chars( const uint8_t *c, int len ){
    write ( port_id, c, len);
}

void Serial::write_string( const char * str ){

    write_chars( (uint8_t*)str, strlen( str ) );
}
