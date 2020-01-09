/*
 * Copyright 2017, Sierra Telecom. All Rights Reserved.
 *
 * This software, associated documentation and materials ("Software"),
 * is owned by Sierra Telecom ("Sierra") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Sierra
 * reserves the right to make changes to the Software without notice. Sierra
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Sierra does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Sierra product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Sierra's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Sierra against all liability.
 */

/** @file i2c_manager.c
 *
 *  Created on: October 3, 2017
 *      Author: greg.phillips
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "common.h"
#include "structs.h"
#include "LTR_329ALS.h"
#include "Si7020.h"
#include "bme680_interface.h"
#include "sgp30_interface.h"
#include "sps30_interface.h"
#include "lux.h"
#include "temp_humidity.h"
#include "co2_voc.h"
#include "i2c_manager.h"
/******************************************************
 *                      Macros
 ******************************************************/
#ifdef PRINT_EV_DEBUGS_FOR_SENSORS
    #undef PRINTF
    #define PRINTF(...) if( ( evc.log_messages & DEBUGS_FOR_SENSORS ) != 0x00 ) printf(__VA_ARGS__)
#elif !defined PRINTF
    #define PRINTF(...)
#endif

#define     cNop        { static int nOP = 3, nOP2; nOP *= 23; nOP2 = nOP -= nOP2; }  /*EO: Nov-5-2018: Added to ease debugging.  Makes a simple no-operation breakpointable code */

/******************************************************
 *                    Constants
 ******************************************************/
#define NO_SPI_I2C_DEVICES      0x04    // Number of devices

#define SAMPLE_STAGGER          200     // Separate each sample 200mS
#define DO_NOT_SAMPLE           0
#define DEFAULT_SAMPLE_RATE     1000    // Check every 1 Second
#define PRIORITY_SAMPLE_RATE    100     // Check very 100 ms
#define ERROR_RESET_MAX         1000    // Check every 100 times thru the loop
#define I2C_TIMEOUT_ERROR       1000     // Timeout if we dont get a response

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef enum {
    SENSOR_RESET_I2C,
    SENSOR_SELECT_INIT,
    SENSOR_SELECT_SENSOR,
    SENSOR_REQUEST,
    SENSOR_WAIT_SENSOR_PROCESSING_TIME,
    SENSOR_READ,
    SENSOR_NEXT,
    SENSOR_REQUEST_COMPLETE,
} i2c_sensor_read_states_t;

typedef struct {
//    wiced_i2c_device_t device;
    uint32_t samples;
    uint32_t sample_rate;
    uint32_t last_sample_time;
    uint32_t sensor_delay;
    uint32_t error_count;
    unsigned int sensor_error   : 1;
    unsigned int enabled        : 1;
} i2c_device_t;

typedef struct {
    i2c_sensor_read_states_t state;
    uint16_t error_reset_count;
    uint8_t device;
    uint32_t write_time;
} i2c_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static void reset_i2c(uint16_t i2c_device );
/******************************************************
 *               Variable Definitions
 ******************************************************/
static i2c_t i2c;

static i2c_device_t i2c_devices[ NO_I2C_DEVICES ];
extern hydra_status_t hs;
const char *i2c_label[ NO_I2C_DEVICES ] =
    {
        "  LUX Sensor: LTR 329ALS     |",
        "  Temperature Sensor: Si7020 |",
        "  CO2 & VOC Sensor: SGP30    |",
        "  Particulate Matter: SPS30  |",
        "  Env. Sensor: BME680        |"
    };
/******************************************************
 *               Function Definitions
 ******************************************************/
void i2c_init_lux(void)
{
    if( init_LTR_329ALS() == true ) {
        PRINTF( "Lux Initialized\r\n" );
        i2c_devices[ I2C_LTR_329ALS ].enabled = true;
        i2c_devices[ I2C_LTR_329ALS ].sensor_error = false;
    } else {
        PRINTF( "Failed to initialize Lux sensor\r\n" );
        reset_i2c( I2C_LTR_329ALS );
        i2c_devices[ I2C_LTR_329ALS ].enabled = false;
        i2c_devices[ I2C_LTR_329ALS ].sensor_error = true;
    }
}
void i2c_init_temp_humidity(void)
{
    if( init_Si7020() == true ) {
        PRINTF( "Temp Initialized\r\n" );
        i2c_devices[ I2C_Si7020 ].enabled = true;
        i2c_devices[ I2C_Si7020 ].sensor_error = false;
    } else {
        PRINTF( "Failed to initialize Temp sensor\r\n" );
        reset_i2c( I2C_Si7020 );
        i2c_devices[ I2C_Si7020 ].enabled = false;
        i2c_devices[ I2C_Si7020 ].sensor_error = true;
    }
}
void i2c_init_CO2_voc(void)
{
    if( init_SGP30() == true )  {
        PRINTF( "CO2/VOC Initialized\r\n" );
        i2c_devices[ I2C_SGP30 ].enabled = true;
        i2c_devices[ I2C_SGP30 ].sensor_error = false;
    } else {
        PRINTF( "Failed to initialize CO2/VOC sensor\r\n" );
        reset_i2c( I2C_SGP30 );
        i2c_devices[ I2C_SGP30 ].enabled = false;
        i2c_devices[ I2C_SGP30 ].sensor_error = true;
    }
}
void i2c_init_particulate_matter(void)
{
    if( init_SPS30() == true )  {
        PRINTF( "Particulate Matter Sensor Initialized\r\n" );
        i2c_devices[ I2C_SPS30 ].enabled = true;
        i2c_devices[ I2C_SPS30 ].sensor_error = false;
    } else {
        PRINTF( "Failed to initialize Particulate Matter sensor\r\n" );
        reset_i2c( I2C_SPS30 );
        i2c_devices[ I2C_SPS30 ].enabled = false;
        i2c_devices[ I2C_SPS30 ].sensor_error = true;
    }
}
void i2c_init_enviromental(void)
{
    if( init_BME680() == true )  {
        PRINTF( "Environmental Sensor Initialized\r\n" );
        i2c_devices[ I2C_BME680 ].enabled = true;
        i2c_devices[ I2C_BME680 ].sensor_error = false;
    } else {
        PRINTF( "Failed to initialize Environmental sensor\r\n" );
        reset_i2c( I2C_BME680 );
        i2c_devices[ I2C_BME680 ].enabled = false;
        i2c_devices[ I2C_BME680 ].sensor_error = true;
    }
}




/**
  * @brief  i2c_init - i2c functions are called on timer based devices - setup a mutex to prevent reentantancy issues.
  * @param  None
  * @retval : None
  */
void i2c_init(void)
{
    i2c_init_lux();
    i2c_init_temp_humidity();
    i2c_init_CO2_voc();
    i2c_init_particulate_matter();
    i2c_init_enviromental();

    printf( "I2C Devices Initialized\r\n" );
}

/**
  * @brief  Initialize the i2c peripherals system
  * @param  None
  * @retval : None
  */
void i2c_manager_init( uint32_t current_time )
{

    /*
     * Start clean
     */
    memset( &i2c_devices, 0x00, sizeof( i2c_devices ) );

    /*
     * Setup each device
     */
    /*
     * LTR_329ALS, - Lux sensor
     */
    /*
    i2c_devices[ I2C_LTR_329ALS ].device.port = WICED_I2C_1;
    i2c_devices[ I2C_LTR_329ALS ].device.address = I2C_LTR_329ALS_ADDRESS;    // External LUX Sensor LTR-329ALS
    i2c_devices[ I2C_LTR_329ALS ].device.address_width = I2C_ADDRESS_WIDTH_7BIT;
    i2c_devices[ I2C_LTR_329ALS ].device.flags = 0;
    i2c_devices[ I2C_LTR_329ALS ].device.speed_mode = I2C_HIGH_SPEED_MODE;
    */
    i2c_devices[ I2C_LTR_329ALS ].samples = 0;
    i2c_devices[ I2C_LTR_329ALS ].sample_rate = DEFAULT_SAMPLE_RATE;
    i2c_devices[ I2C_LTR_329ALS ].last_sample_time = current_time - ( SAMPLE_STAGGER * 2 );
    i2c_devices[ I2C_LTR_329ALS ].sensor_delay = 0;         // Ready to read immediately
    i2c_devices[ I2C_LTR_329ALS ].error_count = 0;
    i2c_devices[ I2C_LTR_329ALS ].sensor_error = false;
    i2c_devices[ I2C_LTR_329ALS ].enabled = true;
    /*
     * Si7020 - Temperature Humidity Sensor
     */
    /*
    i2c_devices[ I2C_Si7020 ].device.port = WICED_I2C_1;
    i2c_devices[ I2C_Si7020 ].device.address = I2C_Si7020_ADDRESS;        // Default for on board Temp / Humidity Sensor Si7020
    i2c_devices[ I2C_Si7020 ].device.address_width = I2C_ADDRESS_WIDTH_7BIT;
    i2c_devices[ I2C_Si7020 ].device.flags = 0;
    i2c_devices[ I2C_Si7020 ].device.speed_mode = I2C_STANDARD_SPEED_MODE,
    */
    i2c_devices[ I2C_Si7020 ].samples = 0;
    i2c_devices[ I2C_Si7020 ].sample_rate = DEFAULT_SAMPLE_RATE;
    i2c_devices[ I2C_Si7020 ].last_sample_time = current_time - ( SAMPLE_STAGGER * 3 );
    i2c_devices[ I2C_Si7020 ].sensor_delay = 0;             // Ready to read immediately
    i2c_devices[ I2C_Si7020 ].error_count = 0;
    i2c_devices[ I2C_Si7020 ].sensor_error = false;
    i2c_devices[ I2C_Si7020 ].enabled = true;
    /*
     * SGP30 - CO2 / VOC Sensor
     */
    /*
    i2c_devices[ I2C_SGP30 ].device.port = WICED_I2C_1;
    i2c_devices[ I2C_SGP30 ].device.address = I2C_SGP30_ADDRESS;
    i2c_devices[ I2C_SGP30 ].device.address_width = I2C_ADDRESS_WIDTH_7BIT;
    i2c_devices[ I2C_SGP30 ].device.flags = 0;
    i2c_devices[ I2C_SGP30 ].device.speed_mode = I2C_STANDARD_SPEED_MODE,
    */
    i2c_devices[ I2C_SGP30 ].samples = 0;
    i2c_devices[ I2C_SGP30 ].sample_rate = DEFAULT_SAMPLE_RATE;
    i2c_devices[ I2C_SGP30 ].last_sample_time = current_time - ( SAMPLE_STAGGER * 4 );
    i2c_devices[ I2C_SGP30 ].sensor_delay = 0;             // Ready to read immediately
    i2c_devices[ I2C_SGP30 ].error_count = 0;
    i2c_devices[ I2C_SGP30 ].sensor_error = false;
    i2c_devices[ I2C_SGP30 ].enabled = true;
    /*
     * SPS30 - Particulate Matter
     */
    /*
    i2c_devices[ I2C_SPS30 ].device.port = WICED_I2C_1;
    i2c_devices[ I2C_SPS30 ].device.address = I2C_SPS30_ADDRESS;
    i2c_devices[ I2C_SPS30 ].device.address_width = I2C_ADDRESS_WIDTH_7BIT;
    i2c_devices[ I2C_SPS30 ].device.flags = 0;
    i2c_devices[ I2C_SPS30 ].device.speed_mode = I2C_STANDARD_SPEED_MODE,
    */
    i2c_devices[ I2C_SPS30 ].samples = 0;
    i2c_devices[ I2C_SPS30 ].sample_rate = DEFAULT_SAMPLE_RATE;
    i2c_devices[ I2C_SPS30 ].last_sample_time = current_time - ( SAMPLE_STAGGER * 5 );
    i2c_devices[ I2C_SPS30 ].sensor_delay = 1000;
    i2c_devices[ I2C_SPS30 ].error_count = 0;
    i2c_devices[ I2C_SPS30 ].sensor_error = false;
    i2c_devices[ I2C_SPS30 ].enabled = true;
    /*
     * BME680 - Environmental Sensor
     */
    /*
    i2c_devices[ I2C_BME680 ].device.port = WICED_I2C_1;
    i2c_devices[ I2C_BME680 ].device.address = I2C_BME680_ADDRESS;
    i2c_devices[ I2C_BME680 ].device.address_width = I2C_ADDRESS_WIDTH_7BIT;
    i2c_devices[ I2C_BME680 ].device.flags = 0;
    i2c_devices[ I2C_BME680 ].device.speed_mode = I2C_STANDARD_SPEED_MODE,
    */
    i2c_devices[ I2C_BME680 ].samples = 0;
    i2c_devices[ I2C_BME680 ].sample_rate = 3000;
    i2c_devices[ I2C_BME680 ].last_sample_time = current_time - ( SAMPLE_STAGGER * 6 );
    i2c_devices[ I2C_BME680 ].sensor_delay = 0;
    i2c_devices[ I2C_BME680 ].error_count = 0;
    i2c_devices[ I2C_BME680 ].sensor_error = false;
    i2c_devices[ I2C_BME680 ].enabled = true;
    /*
     * Initialize all sensors now data structures are setup
     */
    i2c_init();
    i2c.error_reset_count = 0;
    i2c.state = SENSOR_SELECT_INIT;
}

void i2c_sensor_process( uint32_t current_time )
{
#ifdef MODULE_ONLY
    return;
#endif
    bool complete;
    uint16_t debug_factor;

    debug_factor = 1;
    /*
     * If debugging is enabled slow down the sensor collection so it can be monitored
     */
#ifdef PRINT_APOLLO_DEBUGS_FOR_SENSORS
    if( ( fc.log_messages & DEBUGS_FOR_SENSORS ) != 0x00 ) {
        debug_factor = 10;
    }
#endif

    do {
        complete = false;
        switch( i2c.state ) {
            case SENSOR_RESET_I2C :
                i2c_init();
                i2c.state = SENSOR_SELECT_INIT;
                break;
            case SENSOR_SELECT_INIT :
                i2c.device = 0;
                i2c.state = SENSOR_SELECT_SENSOR;
                break;
            case SENSOR_SELECT_SENSOR :
                if( ( i2c_devices[ i2c.device ].enabled == true ) && ( i2c_devices[ i2c.device ].sample_rate != 0 ) &&
                        ( imx_is_later( current_time, i2c_devices[ i2c.device ].last_sample_time +
                        ( i2c_devices[ i2c.device ].sample_rate * debug_factor ) ) == true)  ) {
                    /*
                     * Process this device
                     */
                    i2c_devices[ i2c.device ].last_sample_time = current_time;
                    i2c.state = SENSOR_REQUEST;
                } else
                    i2c.state = SENSOR_NEXT;
                break;
            case SENSOR_REQUEST :
                i2c_devices[ i2c.device ].samples += 1;
                PRINTF( "Sensor Request, Sensor: %u\r\n", i2c.device );
                switch( i2c.device ) {
                    case I2C_LTR_329ALS :
                        if( update_lux() == true ){
                            i2c_devices[ I2C_LTR_329ALS ].sensor_error = false;
                        } else
                            i2c_devices[ I2C_LTR_329ALS ].sensor_error = true;
                        i2c.state = SENSOR_REQUEST_COMPLETE;
                        break;
                    case I2C_Si7020 :
                        if( update_temp_humidity() == true ) {
                            /*
                             * Reset any sensor error
                             */
                            i2c_devices[ I2C_Si7020 ].sensor_error = false;
                        } else {
                            i2c_devices[ I2C_Si7020 ].sensor_error = true;
                        }
                        i2c.state = SENSOR_REQUEST_COMPLETE;
                        break;
                    case I2C_SGP30 :
                        if( update_co2_voc() == true ) {
                            /*
                             * Reset any sensor error
                             */
                            i2c_devices[ I2C_SGP30 ].sensor_error = false;
                        } else {
                            i2c_devices[ I2C_SGP30 ].sensor_error = true;
                        }
                        i2c.state = SENSOR_REQUEST_COMPLETE;
                        break;
                    case I2C_SPS30 :
                        if( start_particulate_matter_measurement() == true ) {
                            /*
                             * Reset any sensor error
                             */
                            i2c_devices[ I2C_SPS30 ].sensor_error = false;
                        } else {
                            i2c_devices[ I2C_SPS30 ].sensor_error = true;
                        }
                        i2c.state = SENSOR_WAIT_SENSOR_PROCESSING_TIME;
                        break;
                    case I2C_BME680 :
                        if( get_environmental_sensor_measurement( current_time ) == true ) {
                            /*
                             * Reset any sensor error
                             */
                            i2c_devices[ I2C_BME680 ].sensor_error = false;
                        } else {
                            i2c_devices[ I2C_BME680 ].sensor_error = true;
                        }
                        i2c.state = SENSOR_WAIT_SENSOR_PROCESSING_TIME;
                        break;

                    default :   // Something wrong here
                        i2c.state = SENSOR_SELECT_INIT;
                        break;
                }
                break;
            case SENSOR_WAIT_SENSOR_PROCESSING_TIME :
                if( imx_is_later( current_time, i2c.write_time + i2c_devices[ i2c.device ].sensor_delay ) )
                    i2c.state = SENSOR_READ;
                break;
            case SENSOR_READ :
                switch( i2c.device ) {
                    case I2C_SPS30 :
                        if( update_particulate_matter() == true ) {
                            i2c_devices[ I2C_SPS30 ].sensor_error = false;
                        } else {
                            i2c_devices[ I2C_SPS30 ].sensor_error = true;
                        }
                        i2c.state = SENSOR_REQUEST_COMPLETE;
                        break;
                default:
                        i2c.state = SENSOR_REQUEST_COMPLETE;
                        break;
                }
                break;
            case SENSOR_REQUEST_COMPLETE :
                i2c_devices[ i2c.device ].last_sample_time = current_time;
                i2c.state = SENSOR_NEXT;
                break;
            case SENSOR_NEXT :
                i2c.device += 1;
                if( i2c.device >= NO_I2C_DEVICES ) {
                    /*
                     * See if we have any sensors in error condition and try to re initialize them
                     */
                    i2c.error_reset_count += 1;
                    if( i2c.error_reset_count >= ERROR_RESET_MAX ) {
                        if( i2c_devices[ I2C_LTR_329ALS ].sensor_error == true ) {
                            i2c_init_lux();
                        }
                        if( i2c_devices[ I2C_Si7020 ].sensor_error == true ) {
                            i2c_init_temp_humidity();
                        }
                        if( i2c_devices[ I2C_SGP30 ].sensor_error == true ) {
                            i2c_init_CO2_voc();
                        }
                        if( i2c_devices[ I2C_SPS30 ].sensor_error == true ) {
                            i2c_init_particulate_matter();
                        }
                        /*
                        if( i2c_devices[ I2C_BME680 ].sensor_error == true ) {
                            i2c_init_enviromental();
                        }
                        */
                        i2c.error_reset_count = 0;
                    }

                    i2c.device = 0;
                }
                i2c.state = SENSOR_SELECT_SENSOR;
                complete = true;
                break;
            default :
                i2c.state = SENSOR_RESET_I2C;
                break;
        }

    } while( complete == false );

}

/**
  * @brief  write some data and then read some data from one of the i2c peripherals connected to the SPI<->I2C interface
  * @param  Perphieral ID, Buffer, count
  * @retval : true/false based on succcess
  */

bool i2c_write_read_sensor( i2c_devices_t i2c_device, uint8_t *tx_data, uint16_t tx_count, uint8_t *rx_data, uint16_t rx_count )
{
    uint16_t i2c_result;
//    wiced_i2c_message_t message;

    if( i2c_device >= NO_I2C_DEVICES)
        return false;
/*
    i2c_result = wiced_i2c_init( &i2c_devices[ i2c_device ].device );

    i2c_result = wiced_i2c_init_combined_message( &message, tx_data, rx_data, tx_count, rx_count, 1, true );
    if( i2c_result != WICED_SUCCESS ) {
        PRINTF( "Failed to initialize combined message, for i2c device %u: result: %u\r\n", i2c_device, i2c_result );
        wiced_i2c_deinit( &i2c_devices[ i2c_device ].device );
        return false;
    }
    i2c_result = wiced_i2c_transfer( &i2c_devices[ i2c_device ].device, &message, 1 );
    if( i2c_result != WICED_SUCCESS ) {
        i2c_devices[ i2c_device ].error_count += 1;
        PRINTF( "Failed to transfer message, for i2c device %u: result: %u\r\n", i2c_device, i2c_result );
        wiced_i2c_deinit( &i2c_devices[ i2c_device ].device );
        return false;
    }
    wiced_i2c_deinit( &i2c_devices[ i2c_device ].device );
*/
    return true;
}


/**
  * @brief  write some data to one of the i2c peripherals connected to the SPI<->I2C interface
  * @param  Perphieral ID, Buffer, count
  * @retval : true/false based on succcess
  */

bool i2c_write_sensor( i2c_devices_t i2c_device, uint8_t *tx_data, uint16_t tx_count )
{
    uint16_t i2c_result;
//    wiced_i2c_message_t message;

    if( i2c_device >= NO_I2C_DEVICES)
        return false;
/*
    i2c_result = wiced_i2c_init( &i2c_devices[ i2c_device ].device );

    i2c_result = wiced_i2c_init_tx_message( &message, tx_data, tx_count, 1, true );
    if( i2c_result != WICED_SUCCESS ) {
        PRINTF( "Failed to initialize combined message, for i2c device %u: result: %u\r\n", i2c_device, i2c_result );
        wiced_i2c_deinit( &i2c_devices[ i2c_device ].device );
        return false;
    }
    i2c_result = wiced_i2c_transfer( &i2c_devices[ i2c_device ].device , &message, 1 );
    if( i2c_result != WICED_SUCCESS ) {
        i2c_devices[ i2c_device ].error_count += 1;
        PRINTF( "Failed to transfer message, for i2c device %u: result: %u\r\n", i2c_device, i2c_result );
        wiced_i2c_deinit( &i2c_devices[ i2c_device ].device );
        return false;
    }
    wiced_i2c_deinit( &i2c_devices[ i2c_device ].device );
*/
    return true;
}
/**
  * @brief  readfrom one of the i2c peripherals connected to the SPI<->I2C interface
  * @param  Perphieral ID, Buffer, count
  * @retval : true/false based on succcess
  */

bool i2c_read_sensor( i2c_devices_t i2c_device, uint8_t *rx_data, uint16_t rx_count )
{
    uint16_t i2c_result;
//    wiced_i2c_message_t message;

    if( i2c_device >= NO_I2C_DEVICES)
        return false;
/*
    i2c_result = wiced_i2c_init( &i2c_devices[ i2c_device ].device );

    i2c_result = wiced_i2c_init_rx_message( &message, rx_data, rx_count, 1, true );
    if( i2c_result != WICED_SUCCESS ) {
        PRINTF( "Failed to initialize combined message, for i2c device %u: result: %u\r\n", i2c_device, i2c_result );
        wiced_i2c_deinit( &i2c_devices[ i2c_device ].device );
        return false;
    }
    i2c_result = wiced_i2c_transfer( &i2c_devices[ i2c_device ].device , &message, 1 );
    if( i2c_result != WICED_SUCCESS ) {
        i2c_devices[ i2c_device ].error_count += 1;
        PRINTF( "Failed to receive message, for i2c device %u: result: %u\r\n", i2c_device, i2c_result );
        wiced_i2c_deinit( &i2c_devices[ i2c_device ].device );
        return false;
    }
    wiced_i2c_deinit( &i2c_devices[ i2c_device ].device );
*/
    return true;
}

/**
  * @brief  print out the controls block for sensor collections
  * @param  void
  * @retval : void
  */

void print_sensor_process(void)
{
    i2c_devices_t i;
    printf( "Scan State: %u Device: %u       __________________________________________________________________________________________\r\n",
            i2c.state, i2c.device );
    printf(    "Sensor Process Status        | Address   | Samples    | Sample Rate   | Last Time   | Delay      | Error Count  | Error |\r\n" );
    for( i = 0; i < NO_I2C_DEVICES; i++ ) {
        if( i2c_devices[ i ].enabled == true ) {
            printf( "%s", i2c_label[ i ] );
            printf( " 0x%02x      | %10lu |    %10lu |  %10lu | %10lu |   %10lu | %s |\r\n",
//                    (uint16_t) i2c_devices[ i ].device.address,
                    i2c_devices[ i ].samples,
                    i2c_devices[ i ].sample_rate,
                    i2c_devices[ i ].last_sample_time,
                    i2c_devices[ i ].sensor_delay,
                    i2c_devices[ i ].error_count,
                    ( i2c_devices[ i ].sensor_error == true ) ? "true "  : "false" );
        } else
            printf( " 0x%02x      | Disabled Sensor                                                      | %s |\r\n",
//                    (uint16_t) i2c_devices[ i ].device.address,
                    ( i2c_devices[ i ].sensor_error == true ) ? "true "  : "false" );
    }

}
/*
 * Return state of sensor error mode
 */
bool sensor_error( sensor_types_t sensor )
{
    switch( sensor ) {
        case SENSOR_HUMIDITY :
            return( i2c_devices[ I2C_Si7020 ].sensor_error );
            break;
        case SENSOR_LUX :
            return( i2c_devices[ I2C_LTR_329ALS ].sensor_error );
            break;
        case SENSOR_TEMPERATURE :
            return( i2c_devices[ I2C_Si7020 ].sensor_error );
            break;
        case SENSOR_CO2_VOC :
            return( i2c_devices[ I2C_SGP30 ].sensor_error );
            break;
        case SENSOR_PARTICULATE_MATTER :
            return( i2c_devices[ I2C_SPS30 ].sensor_error );
            break;
        default :
            return true;    // unknown sensor
    }
}
/**
  * @brief Force SCL and SDA High
  * @param  I2C Device
  * @retval : None
  */
static void reset_i2c(uint16_t i2c_device )
{
    uint16_t i;

    return;
    PRINTF( "Resetting I2C Bus - device: %u failed\r\n", i2c_device );
/*
    wiced_gpio_init( WICED_GPIO_9 , OUTPUT_PUSH_PULL );
    wiced_gpio_init( WICED_GPIO_10 , OUTPUT_PUSH_PULL );
*/
    /*
     * Start Condition
     */
/*
    wiced_gpio_output_high( WICED_GPIO_9  );    // SCL
    wiced_gpio_output_high( WICED_GPIO_10  );   // SDA
    wiced_rtos_delay_milliseconds( 10 );
    wiced_gpio_output_high( WICED_GPIO_9  );    // SCL
    wiced_rtos_delay_milliseconds( 1 );
    wiced_gpio_output_low( WICED_GPIO_10  );    // SDA
*/
    /*
     * Clock 16 0s
     */
/*
    for( i = 0; i < 16; i++ ) {
        wiced_gpio_output_high( WICED_GPIO_9  );    // SCL
        wiced_rtos_delay_milliseconds( 1 );
        wiced_gpio_output_low( WICED_GPIO_9  );    // SCL
        wiced_rtos_delay_milliseconds( 1 );
    }
*/
    /*
     * Stop condition
     */
/*
    wiced_gpio_output_high( WICED_GPIO_9  );    // SCL
    wiced_rtos_delay_milliseconds( 1 );
    wiced_gpio_output_high( WICED_GPIO_10  );   // SDA
*/
}
