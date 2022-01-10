#include "lidar.h"

Lidar::Lidar( void ){}

int Lidar::init( const char* device_str ){

    // Open i2c bus /dev/i2c-1 
    if ((bus_id = i2c_open( device_str )) == -1) {
        // Could not open I2C
        return 0;
    }else{

        i2cport.bus = bus_id;	/* Bus 0 */
        i2cport.addr = 0x62;	/* Slave address is 0x50, 7-bit */
        i2cport.iaddr_bytes = 1;	/* Device internal address is 1 byte */
        i2cport.page_bytes = 8; /* Device are capable of 16 bytes per page */
        return 1;
    }

}

bool Lidar::is_busy( void )
{
    uint8_t data[1]; 

    // Read status register to check busy flag
    int ret = i2c_read( &i2cport, LL40LS_STATUS_REG, data, sizeof(data) );

    // STATUS bit 0 is busyFlag
    uint8_t status = data[0];

    if( status & 0x01 == 1 )
        return true;
    
    return false;
}

uint8_t Lidar::measure( void )
{
    // Build message to tell Lidar to begin measurement
    uint8_t data[] = {LL40LS_MSRREG_ACQUIRE};

    // Write to measure register
    int ret = i2c_write( &i2cport, LL40LS_MEASURE_REG, data, sizeof(data) );

    if( !ret ){
        return LIDAR_ERROR; 
    }

    return LIDAR_OK;
} 


uint8_t Lidar::collect( void ){

    uint8_t data[2];

    // Read two bytes from register 0x0f and 0x10 (autoincrement)
    int ret = i2c_read( &i2cport, LL40LS_DISTHIGH_REG, data, sizeof(data) );

    if( !ret )
        return LIDAR_ERROR;

    // Shift high byte and add to low byte
    distance = (data[0] << 8) | data[1];

    return LIDAR_OK;

}

uint16_t Lidar::get_distance( void ){
    return distance;
}

void Lidar::stop(){
    i2c_close( bus_id );
}