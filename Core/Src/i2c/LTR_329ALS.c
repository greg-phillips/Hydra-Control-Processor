/*
 * $Copyright 2017, Sierra Telecom, Inc.
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Sierra Telecom, Inc.;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Sierra Telecom, Inc.$
 */

/** @file
 *
 *  I2C_LTR_329ALS.c
 *
 *  Created on: September, 2017
 *      Author: greg.phillips
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "stm32g0xx_hal.h"

#include "structs.h"
#include "system.h"
#include "i2c_manager.h"

#include "LTR_329ALS.h"

/******************************************************
 *                      Macros
 ******************************************************/
#ifdef PRINT_EV_DEBUGS_FOR_SENSORS
    #undef PRINTF
	#define PRINTF(...) if( ( evc.log_messages & DEBUGS_FOR_SENSORS ) != 0x00 ) imx_cli_print(__VA_ARGS__)
#elif !defined PRINTF
    #define PRINTF(...)
#endif
/******************************************************
 *                    Constants
 ******************************************************/
#define ALS_CONTR       0x80        // ALS operation mode control SW reset
#define ALS_MEAS_RATE   0x85        // ALS measurement rate in active mode
#define PART_ID         0x86        // Part Number ID and Revision ID
#define MANUFAC_ID      0x87        // Manufacturer ID
#define ALS_DATA_CH1_0  0x88        // ALS measurement CH1 data, lower byte
#define ALS_DATA_CH1_1  0x89        // ALS measurement CH1 data, upper byte
#define ALS_DATA_CH0_0  0x8A        // ALS measurement CH0 data, lower byte
#define ALS_DATA_CH0_1  0x8B        // ALS measurement CH0 data, upper byte
#define ALS_STATUS      0x8C        // ALS new data status
/*
 * Register ALS_CONTR - 0x80
 */
#define ALS_GAIN_1  0x00
#define ALS_GAIN_2  0x04
#define ALS_GAIN_4  0x08
#define ALS_GAIN_8  0x0C
#define ALS_GAIN_48 0x18
#define ALS_GAIN_96 0x1C
#define ALS_GAIN	0x1C

#define ALS_ACTIVE  0x01

#define ALS_INT_100 ( 0x00 )   // Default
#define ALS_INT_50  ( 0x01 )
#define ALS_INT_200 ( 0x02 )
#define ALS_INT_400 ( 0x03 )
#define ALS_INT_150 ( 0x04 )
#define ALS_INT_250 ( 0x05 )
#define ALS_INT_300 ( 0x06 )
#define ALS_INT_350 ( 0x07 )

#define ALS_MES_50      ( 0x00 )
#define ALS_MES 100     ( 0x01 )
#define ALS_MES_200     ( 0x02 )
#define ALS_MES_500     ( 0x03 )    // Default
#define ALS_MES_1000    ( 0x04 )
#define ALS_MES_2000    ( 0x05 )
/*
 * Register 0x8C Bits
 */
#define ALS_DATA_INVALID    0x80
#define ALS_DATA_NEW        0x04
/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
extern hydra_status_t hs;

/******************************************************
 *               Function Declarations
 ******************************************************/
void calc_lux( float *lux_level, float *ir_level, float gain, uint8_t *rx_buffer );
/******************************************************
 *               Variable Definitions
 ******************************************************/

static const  uint8_t als_gain[ 8 ] =
{
    1,
    2,
    4,
    8,
    0,
    0,
    48,
    96
};

static const  float als_val[ 8 ] =
{
    1.0,    // ALS_INT 100   // Default
    0.5,    // ALS_INT_50
    2.0,    // ALS_INT_200
    4.0,    // ALS_INT_400
    1.5,    // ALS_INT_150
    2.5,    // ALS_INT_250
    3.0,    // ALS_INT_300
    3.5    //  ALS_INT_350
};
static int16_t als_time;
static int16_t ignore_sensor;
/******************************************************
 *               Function Definitions
 ******************************************************/
/**
  * @brief  i2c_init_I2C_LTR_329ALS
  * @param  None
  * @retval : None
  */
bool init_LTR_329ALS(void)
{
    bool result;
    uint8_t tx_buffer[ 4 ];
    uint8_t rx_buffer[ 4 ];

    hs.lux_noise_count = 0;
    hs.last_lux_level = 0;
    als_time = ALS_INT_100;
    ignore_sensor = true;

    tx_buffer[ 0 ] = ALS_CONTR;
    tx_buffer[ 1 ] = ALS_ACTIVE;  // Wake up

    result = i2c_write_sensor( I2C_LTR_329ALS , tx_buffer, 2 );
    if( result == false ) {
        printf( "*** LTR-329ALS Wake up Failed\r\n" );
        return false;
    }
    HAL_Delay( 10 );       // Time to be active.

    tx_buffer[ 0 ] = PART_ID;
    if( i2c_write_read_sensor( I2C_LTR_329ALS, tx_buffer, 1, rx_buffer, 1 ) == false ) {
        PRINTF( "LTR 329ALS Failed to read Part ID, result: %d\r\n", result );
        return false;
    }
    printf( "LUX Sensor Part ID: 0x%02X\r\n", rx_buffer[ 0 ] );

    tx_buffer[ 0 ] = MANUFAC_ID;
    if( i2c_write_read_sensor( I2C_LTR_329ALS, tx_buffer, 1, rx_buffer, 1 ) == false ) {
        PRINTF( "LTR 329ALS Failed to read Manufacturing ID, result: %d\r\n", result );
        return false;
    }
    printf( "LUX Sensor Manufacturing ID: 0x%02X\r\n", rx_buffer[ 0 ] );


    tx_buffer[ 0 ] = ALS_CONTR;
    tx_buffer[ 1 ] = ALS_GAIN_1 | ALS_ACTIVE;  // Start with 1X Gain

    result = i2c_write_sensor(  I2C_LTR_329ALS , tx_buffer, 2 );
    if( result == false ) {
        printf( "\r\nLTR-329ALS Set Mode (Gain) Failed: %u\r\n", result );
        return false;
    }

    tx_buffer[ 0 ] = ALS_MEAS_RATE;  // Measurement Rate Register
    als_time = ALS_INT_100; // save for later calculation
    tx_buffer[ 1 ] = ( als_time  << 3  ) | ALS_MES_500;  // Defaults

    result = i2c_write_sensor(  I2C_LTR_329ALS , tx_buffer, 2 );
    if( result == false ) {
        printf( "\r\nLTR-329ALS Config Rate up failed: %u\r\n", result );
        return false;
    }
    ignore_sensor = false;
    return true;
}
/**
  * @brief  I2C_LTR_329ALS_set_mode
  * @param  mode - false - Standby / true - Active
  * @retval : true if result is valid
  */
void LTR_32ALS_set_mode( uint16_t mode )
{
    if( mode == true )
    	ignore_sensor = false;
    else
    	ignore_sensor = true;
}
/**
  * @brief  get_lux_level
  * @param  pointers to lux and ir level integers
  * @retval : true if result is valid
  */

bool LTR_329ALS_get_lux_level( float *lux_level, float *ir_level )
{

    uint16_t result, count;
    uint8_t rx_buffer[ 4 ], tx_buffer[ 4 ], gain;
    // uint8_t old_gain_setting;

    result = false;
    count = 0;

    do {
        tx_buffer[ 0 ] = 0x8C;
        if( i2c_write_read_sensor(  I2C_LTR_329ALS , tx_buffer, 1, rx_buffer, 1 ) == false ) {
            PRINTF( "1)LTR 329ALS I2C write failed, result: %d\r\n" );
            goto fail_read;
        }

        PRINTF( "LTR-329ALS Read 0x8C, Value: 0x%02x\r\n", rx_buffer[ 0 ] );

        if( (rx_buffer[ 0 ] & ALS_DATA_INVALID ) ) {
           //printf( "LTR 329ALS has Bad data: 0x%02x\r\n", rx_buffer[ 0 ] );
           /*
            * Read the bad data and drop it
            */
           tx_buffer[ 0 ] = ALS_DATA_CH1_0;
           if( i2c_write_read_sensor( I2C_LTR_329ALS, tx_buffer, 1, rx_buffer, 4 ) == false ) {
               PRINTF( "2)LTR 329ALS I2C write failed, result: %d\r\n" );
               goto fail_read;
           }

           PRINTF( "LTR-329ALS Read Invalid Data from 0x88, Value: 0x%02x, 0x%02x, 0x%02x, 0x%02x\r\n", rx_buffer[ 0 ], rx_buffer[ 1 ], rx_buffer[ 2 ], rx_buffer[ 3 ] );

           goto return_old_data;
        }

        if( !(rx_buffer[ 0 ] & ALS_DATA_NEW ) ) {
        	PRINTF( "LTR 329ALS Data has been read already, data: 0x%02x\r\n", rx_buffer[ 0 ] );
        	goto return_old_data;
        }

//        old_gain_setting = rx_buffer[ 0 ] & 0x70;

        gain = als_gain[ ( ( rx_buffer[ 0 ] & 0x70 ) >> 4 ) ];    // Read what the div rate should be
        tx_buffer[ 0 ] = ALS_DATA_CH1_0;
        if( i2c_write_read_sensor( I2C_LTR_329ALS, tx_buffer, 1, rx_buffer, 4 ) == false ) {
            PRINTF( "2)LTR 329ALS I2C write failed\r\n" );
            goto fail_read;
        }

        PRINTF( "LTR-329ALS Read ALS_DATA_CH1_0, Values: 0x%02x, 0x%02x, 0x%02x, 0x%02x\r\n", rx_buffer[ 0 ], rx_buffer[ 1 ], rx_buffer[ 2 ], rx_buffer[ 3 ] );

        result = true;
fail_read:
        if( count > 0 )
        	PRINTF( "LUX Bad data count:%u\r\n", count );
        count += 1;

    } while( ( count < 10 ) && ( result == false ) );

    if( result == false ) {
    	/*
    	 * Failing to read SPI Bus, should probably do a reset - more testing before we enable this feature
    	 */
    	return false;
    }
    calc_lux( lux_level, ir_level, gain, rx_buffer );
    /*
     * Check if we have random noise from the LUX sensor - don't want a random glitch to change real data - change can not be > 10% between samples, if it is, then must exist for 5 steps (5 Seconds)
     * Ignore during provisioning as laser activity (lux spikes) are expected
     */
    if( fabs( *lux_level - hs.last_lux_level ) > ( *lux_level * 0.10 ) ) {
        if( hs.lux_noise_count < 5 ) {
            PRINTF( "Ignoring LUX reading: Last valid: %0.2f, Current: %0.2f\r\n", hs.last_lux_level , hs.current_lux_level );
            hs.lux_noise_count += 1;
            goto return_old_data;
        } else {
            hs.lux_noise_count = 0;
        }
    }
    hs.lux_noise_count = 0;
    hs.last_lux_level = *lux_level;
    return( true );
	/*
	 * Use old data
	 */
return_old_data:
	*lux_level = hs.current_lux_level;
	*ir_level = hs.current_ir_lux_level;
	return( true );

}
/*
 * Calculate the LUX based on received values
 */
void calc_lux( float *lux_level, float *ir_level, float gain, uint8_t *rx_buffer )

{
    uint16_t lux_ch0, lux_ch1;
    float PFactor, ratio, als_lux;

    lux_ch1 = ( rx_buffer[ 0 ] | (uint16_t) ( rx_buffer[ 1 ] << 8 ) );

    lux_ch0 = ( rx_buffer[ 2 ] | (uint16_t) ( rx_buffer[ 3 ] << 8 ) );

    /*
     * Calculate the actual lux level from Appendix A
    *
    * RATIO = CH1/(CH0+CH1)
    * IF (RATIO < 0.45)
    *   ALS_LUX = (1.7743 * CH0 + 1.1059 * CH1) / ALS_GAIN / ALS_INT
    * ELSEIF (RATIO < 0.64 && RATIO >= 0.45)
    *   ALS_LUX = (4.2785 * CH0 + 1.9548 * CH1) / ALS_GAIN / ALS_INT
    * ELSEIF (RATIO < 0.85 && RATIO >= 0.64)
    *  ALS_LUX = (0.5926 * CH0 + 0.1185 * CH1) / ALS_GAIN / ALS_INT
    * ELSE
    *   ALS_LUX = 0
    *  END
    *
    */

    PFactor = 1;

    ratio = (float)(lux_ch1) / (float) ( ( lux_ch0 + lux_ch1 ) );
    if( ratio < 0.45 )
        als_lux = ( ( ( ( 1.7743 * (float) lux_ch0 ) + ( 1.1059 * (float) lux_ch1 ) ) / (float) gain ) / als_val[ als_time ] ) * PFactor;
    else if ( ratio < 0.64 && ratio >= 0.45 )
        als_lux = ( ( ( ( 4.2785 * (float) lux_ch0 ) - ( 1.9548 * (float) lux_ch1 ) ) / (float) gain ) / als_val[ als_time ] ) * PFactor;
    else if ( ratio < 0.85 && ratio >= 0.64 )
        als_lux = ( ( ( ( 0.5926 * (float) lux_ch0 ) + ( 0.1185 * (float) lux_ch1 ) ) / (float) gain ) / als_val[ als_time ] ) * PFactor;
    else
        als_lux = 0;

    *lux_level = als_lux;

    if( ratio < 0.45 )
        als_lux = ( ( ( ( 1.7743 * (float) lux_ch0 ) + ( 1.1059 * (float) lux_ch1 ) ) / (float) gain ) / als_val[ als_time ] );
    else if ( ratio < 0.64 && ratio >= 0.45 )
        als_lux = ( ( ( ( 4.2785 * (float) lux_ch0 ) - ( 1.9548 * (float) lux_ch1 ) ) / (float) gain ) / als_val[ als_time ] );
    else if ( ratio < 0.85 && ratio >= 0.64 )
        als_lux = ( ( ( ( 0.5926 * (float) lux_ch0 ) + ( 0.1185 * (float) lux_ch1 ) ) / (float) gain ) / als_val[ als_time ] );
    else
        als_lux = 0;

    *ir_level = als_lux;

    PRINTF( "Light Level Lux: %0.4f, Ch 0: %d, Ch 1: %d, Gain: %u\r\n", *lux_level, lux_ch0, lux_ch1, gain );

    /*
     * Check to see if we adjust the gain
    tx_buffer[ 0 ] = ALS_CONTR;

    if( *lux_level < 600 )
        tx_buffer[ 1 ] = ALS_GAIN_96 | ALS_ACTIVE;  // Use 96X Gain
    else if( *lux_level < 1300 )
        tx_buffer[ 1 ] = ALS_GAIN_48 | ALS_ACTIVE;  // Use 48X Gain
    else if( *lux_level < 8000 )
        tx_buffer[ 1 ] = ALS_GAIN_8 | ALS_ACTIVE;  // Use 8X Gain
    else if( *lux_level < 16000 )
        tx_buffer[ 1 ] = ALS_GAIN_4 | ALS_ACTIVE;  // Use 4X Gain
    else if( *lux_level < 32000 )
        tx_buffer[ 1 ] = ALS_GAIN_2 | ALS_ACTIVE;  // Use 2X Gain
    else
        tx_buffer[ 1 ] = ALS_GAIN_1 | ALS_ACTIVE;  // Use 1X Gain

    if( ( old_gain_setting >> 2 ) != ( tx_buffer[ 1 ] & ALS_GAIN ) ) {  // Only update when we need to
        printf( "Updating Gain from: 0x%02x to: 0x%02x\r\n", (uint16_t) old_gain_setting, (uint16_t) ( tx_buffer[ 1 ] & ALS_GAIN ) );
        wiced_result = i2c_write_sensor(  I2C_LTR_329ALS , tx_buffer, 2 );
        if( wiced_result != WICED_SUCCESS ) {
            printf( "LTR-329ALS Set Mode (Gain) Failed, result: %d\r\n", wiced_result );
        }
    }
     */

}
