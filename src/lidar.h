#include <iostream>
#include "i2c/i2c.h"


/* Configuration Constants */
static const uint8_t LL40LS_BASEADDR              = 0x62; /* 7-bit address */
static const uint8_t LL40LS_SIG_COUNT_VAL_DEFAULT = 0x80; /* Default maximum acquisition count */

/* LL40LS Registers addresses */
static const uint8_t LL40LS_MEASURE_REG           = 0x00; /* Measure range register */
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

static const float LL40LS_MIN_DISTANCE = 0.05f;
static const float LL40LS_MAX_DISTANCE = 35.00f;

// Normal conversion wait time.
static const uint32_t LL40LS_CONVERSION_INTERVAL = 50; // ms
// Maximum time to wait for a conversion to complete.
static const uint32_t LL40LS_CONVERSION_TIMEOUT = 100; // ms


class Lidar 
{
public:

	Lidar( const char* device );

	int init();

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

protected:

	int measure();
    
    int collect();

	/**
	 * Reset the sensor to power on defaults plus additional configurations.
	 */
	int reset_sensor();

};