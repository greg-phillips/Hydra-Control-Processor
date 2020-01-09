/*
 * Copyright 2019, iMatrix Systems, Inc.. All Rights Reserved.
 *
 * This software, associated documentation and materials ("Software"),
 * is owned by iMatrix Systems ("iMatrix") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. iMatrix
 * reserves the right to make changes to the Software without notice. iMatrix
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. iMatrix does
 * not authorize its products for use in any products where a malfunction or
 * failure of the iMatrix product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including iMatrix's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify iMatrix against all liability.
 */

/** @file bme_interface.c
 *
 *  Created on: November 06, 2019
 *      Author: greg.phillips
 * H = 44330 * [1 - (P/p0)^(1/5.255) ]
 * H = altitude (m)
P = measured pressure (Pa) from the sensor
p0 = reference pressure at sea level (e.g. 1013.25hPa)
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <memory.h>

#include "system.h"
#include "structs.h"
#include "stm32g0xx_hal.h"

#include "bme680.h"
#include "bsec_datatypes.h"
#include "bsec_interface.h"
#include "imx_bsec_integration.h"
#include "bsec_integration.h"
#include "bme680_defs.h"
/******************************************************
 *                      Macros
 ******************************************************/
#ifdef PRINT_DEBUGS_FOR_SENSORS
    #undef PRINTF
    #define PRINTF(...) if( ( hs.log_messages & DEBUGS_FOR_SENSORS ) != 0x00 ) imx_cli_print(__VA_ARGS__)
#elif !defined PRINTF
    #define PRINTF(...)
#endif

/******************************************************
 *                    Constants
 ******************************************************/
#define BME680_DEFAULT_SAMPLE_RATE      ( BSEC_SAMPLE_RATE_LP )
#define BME680_SAVE_STATE_INTERVAL      ( 5 )
/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
void sleep(uint32_t t_ms);
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer);
void state_save(const uint8_t *state_buffer, uint32_t length);
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer);
void output_ready( int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
     float static_iaq, float co2_equivalent, float breath_voc_equivalent );
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len);
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len);
/******************************************************
 *               Variable Definitions
 ******************************************************/
/* Global sensor APIs data structure */
struct bme680_dev bme680_g;

/* Global temperature offset to be subtracted */
float bme680_temperature_offset_g = 0.0f;
static bool bms680_ready_request_pending = true;
extern hydra_status_t hs;

/******************************************************
 *               Function Definitions
 ******************************************************/
/**
  * @brief Initialize the BME Sensor
  * @param  None
  * @retval : True / False
  */
bool init_BME680(void)
{
    /*!
     * @brief       Initialize the BME680 sensor and the BSEC library
     *
     * @param[in]   sample_rate         mode to be used (either BSEC_SAMPLE_RATE_ULP or BSEC_SAMPLE_RATE_LP)
     * @param[in]   temperature_offset  device-specific temperature offset (due to self-heating)
     *
     * @return      zero if successful, negative otherwise
     */
    uint8_t bsec_state[ BSEC_MAX_PROPERTY_BLOB_SIZE ] = {0};
    uint8_t bsec_config[ BSEC_MAX_PROPERTY_BLOB_SIZE ] = {0};
    uint8_t work_buffer[ BSEC_MAX_PROPERTY_BLOB_SIZE ] = {0};
    int bsec_state_len, bsec_config_len;

    return_values_init ret = {BME680_OK, BSEC_OK};

    /* Fixed I2C configuration */
    bme680_g.dev_id = BME680_I2C_ADDR_PRIMARY;
    bme680_g.intf = BME680_I2C_INTF;
    /* User configurable I2C configuration */
    bme680_g.write = bus_write;
    bme680_g.read = bus_read;
    bme680_g.delay_ms = sleep;

    /*
     * Initialize BME680 API
     * */
    ret.bme680_status = bme680_init( &bme680_g );
    if (ret.bme680_status != BME680_OK) {
        printf( "BME680 Failed to Initialize: %d\r\n", (int16_t) ret.bme680_status );
        return false;
    }

    /*
     * Initialize BSEC library
     */
    ret.bsec_status = bsec_init();
    if (ret.bsec_status != BSEC_OK) {
        printf( "BME680 Failed to Initialize BSEC, Status: %d, BSEC Status: %d\r\n", (int16_t) ret.bme680_status, (int16_t) ret.bsec_status );
        return false;
    }

    /*
     * Load library config, if available
     */
    bsec_config_len = config_load( bsec_config, sizeof( bsec_config ) );
    if( bsec_config_len != 0 ) {
        ret.bsec_status = bsec_set_configuration( bsec_config, bsec_config_len, work_buffer, sizeof(work_buffer) );
        if (ret.bsec_status != BSEC_OK) {
            printf( "BME680 Failed to set BSC Configuration, Status: %d, BSEC Status: %d\r\n", (int16_t) ret.bme680_status, (int16_t) ret.bsec_status );
            return false;
        }
    }

    /*
     * Load previous library state, if available
     */
    bsec_state_len = state_load( bsec_state, sizeof( bsec_state ) );
    if ( bsec_state_len != 0 ) {
        ret.bsec_status = bsec_set_state( bsec_state, bsec_state_len, work_buffer, sizeof( work_buffer ) );
        if ( ret.bsec_status != BSEC_OK ) {
            printf( "BME680 Failed to set BSC State, Status: %d, BSEC Status: %d\r\n", (int16_t) ret.bme680_status, (int16_t) ret.bsec_status );
            return false;
        }
    }

    /*
     * Set temperature offset
     */
    bme680_temperature_offset_g = EV_TEMP_OFFSET;

    /*
     * Call to the function which sets the library with subscription information
     */
    ret.bsec_status = bme680_bsec_update_subscription( BME680_DEFAULT_SAMPLE_RATE );
    if( ret.bsec_status != BSEC_OK ) {
        printf( "BME68cnn0 Failed to set BSC Subscription, Status: %d, BSEC Status: %d\r\n", (int16_t) ret.bme680_status, (int16_t) ret.bsec_status );
        return false;
    }

    return true;
}
/*
 * Allocate enough memory for up to BSEC_MAX_PHYSICAL_SENSOR physical inputs
 * Add These to CCSRAM
 */
bsec_input_t bsec_inputs[ BSEC_MAX_PHYSICAL_SENSOR ];

/* Number of inputs to BSEC */
uint8_t num_bsec_inputs = 0;

/* BSEC sensor settings struct */
bsec_bme_settings_t sensor_settings;

/* Save state variables */
uint8_t bsec_state[ BSEC_MAX_STATE_BLOB_SIZE ];
uint8_t work_buffer[ BSEC_MAX_STATE_BLOB_SIZE ];
uint32_t bsec_state_len = 0;
uint32_t n_samples = 0;

bool get_environmental_sensor_measurement( uint32_t current_time )
{

    int64_t time_stamp;

    bsec_library_return_t bsec_status = BSEC_OK;

    /*
     * Get the timestamp in nanoseconds before calling bsec_sensor_control()
     */
    time_stamp = (uint64_t) current_time * 1000000;

    /* Retrieve sensor settings to be used in this time instant by calling bsec_sensor_control */
    bsec_sensor_control( time_stamp, &sensor_settings );

    /* Trigger a measurement if necessary */
    bme680_bsec_trigger_measurement( &sensor_settings, sleep );

    /* Read data from last measurement */
    num_bsec_inputs = 0;
    bme680_bsec_read_data(time_stamp, bsec_inputs, &num_bsec_inputs, sensor_settings.process_data);

    /* Time to invoke BSEC to perform the actual processing */
    bme680_bsec_process_data(bsec_inputs, num_bsec_inputs, output_ready);

    /* Increment sample counter */
    n_samples++;

    /* Retrieve and store state if the passed save_intvl */
    if ( n_samples >= BME680_SAVE_STATE_INTERVAL )  {
        bsec_status = bsec_get_state(0, bsec_state, sizeof(bsec_state), work_buffer, sizeof(work_buffer), &bsec_state_len);
        if (bsec_status == BSEC_OK)  {
            state_save( bsec_state, bsec_state_len );
        }
        n_samples = 0;
    }
    /*
     * Data was stored in callback function as argument to bme680_bsec_process_data
     */
    return true;
}

/*
 * Copyright (C) 2017 Robert Bosch. All Rights Reserved.
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet.  Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchasers own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 */

/*!
 * @file bsec_iot_example.c
 *
 * @brief
 * Example for using of BSEC library in a fixed configuration with the BME680 sensor.
 * This works by running an endless loop in the bsec_iot_loop() function.
 */

/*!
 * @addtogroup bsec_examples BSEC Examples
 * @brief BSEC usage examples
 * @{*/

/**********************************************************************************************************************/
/* header files */
/**********************************************************************************************************************/

#include "bsec_integration.h"
#include "i2c_manager.h"
/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/

/*!
 * @brief           Write operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    // ...
    // Please insert system specific function to write to the bus where BME680 is connected
    // ...
    uint8_t tmp_buff[ BME680_TMP_BUFFER_LENGTH + 1 ] = { 0 };

    if( data_len > BME680_TMP_BUFFER_LENGTH + 1 )
        printf( "BME680 I2C Buffer exceeds maximum: %u\r\n", data_len );
    else {
        tmp_buff[ 0 ] = reg_addr;
        memcpy( &tmp_buff[ 1 ], reg_data_ptr, data_len );
        if( i2c_write_sensor( I2C_BME680, tmp_buff, data_len + 1 ) == true )
            return BSEC_OK;
        else
            return BME680_E_COM_FAIL;
    }
    return BME680_E_COM_FAIL;
}

/*!
 * @brief           Read operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 *
 * @return          result of the bus communication function
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    // ...
    // Please insert system specific function to read from bus where BME680 is connected
    // ...
    if( i2c_write_sensor( I2C_BME680, &reg_addr, 1 ) == true ) {
        if( i2c_read_sensor( I2C_BME680, reg_data_ptr, data_len ) == true )
            return BSEC_OK;
        else
            return BME680_E_COM_FAIL;

    } else
        return BME680_E_COM_FAIL;
    return BME680_E_COM_FAIL;
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_ms    time in milliseconds
 *
 * @return          none
 */
void sleep(uint32_t t_ms)
{
    // ...
    // Please insert system specific function sleep or delay for t_ms milliseconds
    // ...
    HAL_Delay( t_ms );
}

/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
    uint32_t current_time;
    int64_t system_current_time = 0;
    // ...
    // Please insert system specific function to retrieve a timestamp (in microseconds)
    // ...
    current_time = HAL_GetTick();
    system_current_time = (uint64_t) current_time * 1000;
    return system_current_time;
}

/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp       time in nanoseconds
 * @param[in]       iaq             IAQ signal
 * @param[in]       iaq_accuracy    accuracy of IAQ signal
 * @param[in]       temperature     temperature signal
 * @param[in]       humidity        humidity signal
 * @param[in]       pressure        pressure signal
 * @param[in]       raw_temperature raw temperature signal
 * @param[in]       raw_humidity    raw humidity signal
 * @param[in]       gas             raw gas sensor signal
 * @param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * @return          none
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
     float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{
    float altitude;

    // ...
    // Please insert system specific code to further process or display the BSEC outputs
    // ...
    altitude = 44330 * (1 - (pow( ( pressure / 1013.25 ), (1/5.255) ) ) );
    /*
    printf( "BME680: Status: %u, IAQ: %f, IAQ Accuracy %u, Temperature: %f, Humidity: %f, Pressure: %f, Altitude: %f, raw_temperature: %f, raw_humidity: %f, gas: %f, Equiv CO2: %f, Breath VOC: %f\r\n",
            bsec_status, iaq, (uint16_t) iaq_accuracy, temperature, humidity, pressure, altitude, raw_temperature, raw_humidity, gas, co2_equivalent, breath_voc_equivalent );
            */
    /*
     * Update current settings
     */
    hs.current_iaq = iaq;
    hs.current_iaq_accuracy = iaq_accuracy;
    hs.current_temperature = temperature;
    hs.current_humidity = humidity;
    hs.current_pressure = pressure;
    hs.current_altitude = altitude;
    hs.current_raw_temperature = raw_temperature;
    hs.current_raw_humidity = raw_humidity;
    hs.current_gas = gas;
    hs.current_co2_equivalent = co2_equivalent;
    hs.current_breath_voc_equivalent = breath_voc_equivalent;
    hs.valid_environmental_sensor = true;
    bms680_ready_request_pending = false;   // Got the data its good
}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available,
    // otherwise return length of loaded state string.
    // ...
    if( hs.bsec_state_valid == true ) {
        if( n_buffer > BSEC_MAX_PROPERTY_BLOB_SIZE )
            return 0;

        memset( &hs.bsec_state, 0x00, BSEC_MAX_PROPERTY_BLOB_SIZE );
        memcpy( state_buffer, &hs.bsec_state, n_buffer );
        return n_buffer;
    } else
        return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
    printf( "BME680 Requesting: %lu Bytes for State storage\r\n", length );
    if( length > BSEC_MAX_PROPERTY_BLOB_SIZE )
        return;

    memcpy( &hs.bsec_state, state_buffer, length );
    hs.bsec_state_valid = true;
}

/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available,
    // otherwise return length of loaded config string.
    // ...
    printf( "BME680 Requesting: %lu Bytes of library Config\r\n", n_buffer );
    if( n_buffer != BSEC_MAX_PROPERTY_BLOB_SIZE )
        return 0;

    memcpy( config_buffer, &hs.bsec_config, BSEC_MAX_PROPERTY_BLOB_SIZE );
    printf( "BSEC Config Loaded from internal configuration\r\n" );
    return 454;
}


/*! @}*/

