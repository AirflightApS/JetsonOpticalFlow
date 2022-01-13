#include <iostream>
#include "i2c/i2c.h"

#define LIDAR_OK 0
#define LIDAR_ERROR -1

/* Configuration Constants */
static const uint8_t LL40LS_BASEADDR              = 0x62; /* 7-bit address */
static const uint8_t LL40LS_SIG_COUNT_VAL_DEFAULT = 0x80; /* Default maximum acquisition count */

/* LL40LS Registers addresses */
static const uint8_t LL40LS_MEASURE_REG           = 0x00; /* Measure range register */
static const uint8_t LL40LS_STATUS_REG            = 0x01; /* Measure range register */
static const uint8_t LL40LS_MSRREG_RESET          = 0x00; /* reset to power on defaults */
static const uint8_t LL40LS_MSRREG_ACQUIRE        = 0x04; /* Value to acquire a measurement, version specific */
static const uint8_t LL40LS_DISTHIGH_REG          = 0x0F; /* High byte of distance register, auto increment */
static const uint8_t LL40LS_AUTO_INCREMENT        = 0x80;
static const uint8_t LL40LS_HW_VERSION            = 0x41;
static const uint8_t LL40LS_SW_VERSION            = 0x4f;
static const uint8_t LL40LS_SIGNAL_STRENGTH_REG   = 0x0e;
static const uint8_t LL40LS_PEAK_STRENGTH_REG     = 0x0c;
static const uint8_t LL40LS_UNIT_ID_HIGH          = 0x16;
static const uint8_t LL40LS_UNIT_ID_LOW           = 0x17;

static const uint8_t LL40LS_SIG_COUNT_VAL_REG     = 0x02; /* Maximum acquisition count register */
static const uint8_t LL40LS_SIG_COUNT_VAL_MAX     = 0xFF; /* Maximum acquisition count max value */

static const int LL40LS_SIGNAL_STRENGTH_MIN_V3HP  = 70;  /* Min signal strength for V3HP */
static const int LL40LS_SIGNAL_STRENGTH_MAX_V3HP  = 255; /* Max signal strength for V3HP */

static const int LL40LS_SIGNAL_STRENGTH_LOW       = 24;  /* Minimum signal strength for a valid measurement */
static const int LL40LS_PEAK_STRENGTH_LOW         = 135; /* Minimum peak strength for accepting a measurement */
static const int LL40LS_PEAK_STRENGTH_HIGH        = 234; /* Max peak strength raw value */

static const int LL40LS_MIN_DISTANCE = 10; // cm
static const int LL40LS_MAX_DISTANCE = 3500; // cm
static const float LL40LS_FOV = 0.008; // Divergence 8 mRadian

// Normal conversion wait time.
static const uint32_t LL40LS_CONVERSION_INTERVAL = 50; // ms
// Maximum time to wait for a conversion to complete.
static const uint32_t LL40LS_CONVERSION_TIMEOUT = 100; // ms


class Lidar 
{
public:

	Lidar(void);

	int init( const char* device_str );

	/**
	 * @brief Read and return result of distance measurement.
	 * 
	 * @return uint8_t  
	 */
	uint8_t collect( void );

	/**
	 * @brief Initiate a distance measurement by writing to register 0x00.
	 * 
	 * @return uint8_t  
	 */
	uint8_t measure( void );

	/**
	 * @brief Read BUSY flag from device registers. Function will return 0x00 if not busy.
	 * 
	 * @return true 
	 * @return false 
	 */
	bool is_busy( void );

	void stop( void );

	/**
	 * @brief Get the distance (should only be called after a successful measurements is acquired)
	 * 
	 * @return uint16_t 
	 */
	uint16_t get_distance( void );

	uint8_t get_quality( void ); 

private:

	// Configure I2C
    I2CDevice i2cport;
	int bus_id;

	uint16_t distance;
	uint8_t signal_quality;

	/**
	 * Reset the sensor to power on defaults plus additional configurations.
	 */
	int reset_sensor();

};