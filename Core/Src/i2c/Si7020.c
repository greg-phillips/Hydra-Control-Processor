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
 *  Si7020.c
 *
 *  Created on: September, 2017
 *      Author: greg.phillips
 *
 */


#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx_hal.h"

#include "structs.h"
#include "system.h"
#include "i2c_manager.h"

#include "Si7020.h"

/******************************************************
 *                      Macros
 ******************************************************/
#ifdef PRINT_DEBUGS_FOR_SENSORS
    #undef PRINTF
	#define PRINTF(...) if( ( evc.log_messages & DEBUGS_FOR_SENSORS ) != 0x00 ) imx_cli_print(__VA_ARGS__)
#elif !defined PRINTF
    #define PRINTF(...)
#endif
/******************************************************
 *                    Constants
 ******************************************************/

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

/******************************************************
 *               Variable Definitions
 ******************************************************/
extern hydra_status_t hs;

/******************************************************
 *               Function Definitions
 ******************************************************/
uint8_t Si_crc( uint8_t *bytes, uint16_t length );
uint8_t Si_crc8( uint8_t value, uint8_t seed );
/**
  * @brief  init_Si7020
  * @param  None
  * @retval : None
  */
bool init_Si7020(void)
{
    bool result;
    uint8_t tx_buffer[ 4 ], rx_buffer[ 8 ];
    float temp, humidity;

    PRINTF( "Setting up I2C for Si7020\r\n" );

    /*
     * Set up the config for the the Silicon Labs Temp and Humidity Sensor
     */
    tx_buffer[ 0 ] = 0xE6;      // Write RH/T User Register
    tx_buffer[ 1 ] = 0x00;      // 12 Bit RH & 14 Bit Temp - No Heater
    result = i2c_write_sensor( I2C_Si7020, tx_buffer, 2 );
    if( result != true ) {
        PRINTF( "\r\nSi7020 I2C transmit Failed\r\n" );
        return false;
    }
    /*
     * Read the serial number
     */
    tx_buffer[ 0 ] = 0xFA;      // Read from First Byte of ID
    tx_buffer[ 1 ] = 0x0F;      //
    result = i2c_write_sensor( I2C_Si7020, tx_buffer, 2 );
    if( result != true ) {
        PRINTF( "\r\nSi7020 I2C transmit Failed\r\n" );
        return false;
    }

    result = i2c_read_sensor( I2C_Si7020, rx_buffer, 8 );
    if( result != true ) {
        PRINTF( "\r\nSi7020 I2C read Serial Number\r\n" );
        return false;
    }
    PRINTF( "Temperature Sensor serial number output: %02X%02X%02X%02X%02X%02X%02X%02X", rx_buffer[ 0 ], rx_buffer[ 1 ], rx_buffer[ 2 ], rx_buffer[ 3 ],
    		rx_buffer[ 4 ], rx_buffer[ 5 ], rx_buffer[ 6 ], rx_buffer[ 7 ] );

    tx_buffer[ 0 ] = 0xFC;      // Read from 2nd Byte of ID
    tx_buffer[ 1 ] = 0xC9;      //
    result = i2c_write_sensor( I2C_Si7020, tx_buffer, 2 );
    if( result != true ) {
        PRINTF( "\r\nSi7020 I2C transmit Failed\r\n" );
        return false;
    }

    result = i2c_read_sensor( I2C_Si7020, rx_buffer, 6 );
    if( result != true ) {
        PRINTF( "\r\nSi7020 I2C read Serial Number\r\n" );
        return false;
    }
    PRINTF( "-%02X%02X%02X%02X%02X%02X\r\n", rx_buffer[ 0 ], rx_buffer[ 1 ], rx_buffer[ 2 ], rx_buffer[ 3 ],
    		rx_buffer[ 4 ], rx_buffer[ 5 ] );
    if( Si7020_get_temp( &temp, &humidity ) == false )
        return false;
    /*
     * Display if in debug mode
     */
    PRINTF( "I2C Si7020 Temp: %0.02fC, Humidity: %0.02f%%RH\r\n", temp, humidity );
    return true;
}
uint16_t Si7020_heater( uint16_t state )
{
    bool result;
    uint8_t tx_buffer[ 4 ];

	if( state == true ) {
	    tx_buffer[ 0 ] = 0xE6;      // Write RH/T User Register
	    tx_buffer[ 1 ] = 0x04;      // 12 Bit RH & 14 Bit Temp - Heater ON
	    result = i2c_write_sensor( I2C_Si7020, tx_buffer, 2 );
	    if( result != true ) {
	        //printf( "\r\nSi7020 I2C transmit Failed\r\n" );
	        return false;
	    }
	    tx_buffer[ 0 ] = 0x51;      // Current
	    tx_buffer[ 1 ] = 0x0F;      // 94mA
	    result = i2c_write_sensor( I2C_Si7020, tx_buffer, 2 );
	    if( result != true ) {
	        //printf( "\r\nSi7020 I2C transmit Failed\r\n" );
	        return false;
	    }
	} else {
	    tx_buffer[ 0 ] = 0xE6;      // Write RH/T User Register
	    tx_buffer[ 1 ] = 0x00;      // 12 Bit RH & 14 Bit Temp - Heater OFF
	    result = i2c_write_sensor( I2C_Si7020, tx_buffer, 2 );
	    if( result != true ) {
	        //printf( "\r\nSi7020 I2C transmit Failed\r\n" );
	        return false;
	    }

	}
	return true;
}
uint16_t Si7020_get_serial( uint8_t serial_number[ 8 ] )
{
    bool result;
    uint8_t tx_buffer[ 4 ], rx_buffer[ 8 ];
    /*
     * Read the serial number
     */
    tx_buffer[ 0 ] = 0xFA;      // Read from First Byte of ID
    tx_buffer[ 1 ] = 0x0F;      //
    result = i2c_write_sensor( I2C_Si7020, tx_buffer, 2 );
    if( result != true ) {
        //printf( "\r\nSi7020 I2C transmit Failed\r\n" );
        return false;
    }

    result = i2c_read_sensor( I2C_Si7020, rx_buffer, 8 );
    if( result != true ) {
        //printf( "\r\nSi7020 I2C read Serial Number\r\n" );
        return false;
    }
    PRINTF( "Serial number output: %02X%02X%02X%02X%02X%02X%02X%02X\r\n", rx_buffer[ 0 ], rx_buffer[ 1 ], rx_buffer[ 2 ], rx_buffer[ 3 ],
    		rx_buffer[ 4 ], rx_buffer[ 5 ], rx_buffer[ 6 ], rx_buffer[ 7 ] );

    // Check the CRC
    uint8_t crc = 0;
    uint16_t i;
    for(i = 0; i < 4; i++) {
        crc =Si_crc8( rx_buffer[ 2*i ], crc );
        if(crc != rx_buffer[ 2*i +1 ]) {
            //printf( "Checksum on serial number failed\r\n");
            // return false; - REfer to reference code - https://developer.mbed.org/users/kgills/code/Si7020/file/eca9d19c17ab/Si7020.cpp
        }
    }

    serial_number[ 7 ] = rx_buffer[ 0 ];
    serial_number[ 6 ] = rx_buffer[ 2 ];
    serial_number[ 5 ] = rx_buffer[ 4 ];
    serial_number[ 4 ] = rx_buffer[ 6 ];

    tx_buffer[ 0 ] = 0xFC;      // Read from 2nd Byte of ID
    tx_buffer[ 1 ] = 0xC9;      //
    result = i2c_write_sensor( I2C_Si7020, tx_buffer, 2 );
    if( result != true ) {
        //printf( "\r\nSi7020 I2C transmit Failed\r\n" );
        return false;
    }

    result = i2c_read_sensor( I2C_Si7020, rx_buffer, 6 );
    if( result != true ) {
        //printf( "\r\nSi7020 I2C read Serial Number\r\n" );
        return false;
    }
    PRINTF( "Serial number output: %02X%02X%02X%02X%02X%02X\r\n", rx_buffer[ 0 ], rx_buffer[ 1 ], rx_buffer[ 2 ], rx_buffer[ 3 ],
    		rx_buffer[ 4 ], rx_buffer[ 5 ] );
    // Check the CRC
     crc = 0;
     for(i = 0; i < 2; i++) {
         crc =Si_crc8( rx_buffer[ 3*i ], crc);
         crc =Si_crc8( rx_buffer[ 3*i + 1 ], crc);
         if(crc != rx_buffer[ 3*i + 2 ]) {
         	//printf( "Failed Second Checksum\r\n" );
         	// return false;
         }
     }

     serial_number[ 3 ] = rx_buffer[ 0 ];
     serial_number[ 2 ] = rx_buffer[ 1 ];
     serial_number[ 1 ] = rx_buffer[ 3 ];
     serial_number[ 0 ] = rx_buffer[ 4 ];
     return true;
}
/**
  * @brief  Si7020_update_temp
  * @param  None
  * @retval : None
  */

bool Si7020_get_temp(float *temp, float *humidity)
{
    bool result;
    uint16_t calc;
    uint8_t rx_buffer[ 4 ], tx_buffer[ 4 ], crc;

    /*
     * Read the Temp and Humidity Sensor
     */

    tx_buffer[ 0 ] = 0xF5;      // Read RH first then Temp should be available without delay
    result = i2c_write_sensor( I2C_Si7020, tx_buffer, 1 );
    if( result != true ) {
        PRINTF( "\r\nSi7020 I2C write Failed\r\n" );
        return false;
    }

    HAL_delay( 25 );    // Wait for device to process

    result = i2c_read_sensor( I2C_Si7020, rx_buffer, 3 );
    if( result != true ) {
        PRINTF( "\r\nSi7020 I2C read humidity Failed\r\n" );
        return false;
    }

    crc = Si_crc( &rx_buffer[ 0 ], 2 );
    PRINTF( "Humidity Read: 0x%02x, 0x%02x, 0x%02x CRC: 0x%02x\r\n", rx_buffer[ 0 ], rx_buffer[ 1 ], rx_buffer[ 2 ], crc );

    if( rx_buffer[ 2 ] == crc ) {  // Verify CRC
        calc = ( (uint16_t)( rx_buffer[ 0 ] ) << 8 ) | (uint16_t) rx_buffer[ 1 ];
        *humidity = ( ( 125.0 * (float) calc ) / 65536.0 ) - 6;    // From data sheet
        /*
         *  Check if this passes the sniff test
         */
        if( *humidity > 100)
            *humidity  = 100.0;  // Check for out of range values
        else if ( *humidity < 0.0 )
            *humidity = 0.0;

        tx_buffer[ 0 ] = 0xF3;      // Read Temp from last RH reading
        result = i2c_write_sensor( I2C_Si7020, tx_buffer, 1 );
        if( result != true ) {
            PRINTF( "\r\nSi7020 I2C write Failed\r\n" );
            return false;
        }

        HAL_Delay( 25 );    // Wait for device to process

        result = i2c_read_sensor( I2C_Si7020, rx_buffer, 3 );
        if( result != true ) {
            PRINTF( "\r\nSi7020 I2C read temp Failed\r\n" );
            return false;
        }

        crc = Si_crc( &rx_buffer[ 0 ], 2 );
        PRINTF( "Temp Read: 0x%02x, 0x%02x, 0x%02x CRC: 0x%02x\r\n", rx_buffer[ 0 ], rx_buffer[ 1 ], rx_buffer[ 2 ], Si_crc( &rx_buffer[ 0 ], 2 ) );

        if( rx_buffer[ 2 ] == crc ) {  // Verify CRC
            calc = ( (uint16_t)( rx_buffer[ 0 ] ) << 8 ) | (uint16_t) rx_buffer[ 1 ];
            *temp = ( ( 175.72 * (float) calc ) / 65536.0 ) - 46.85;

            PRINTF( "I2C Si7020 Temp: %0.02fC, Humidity: %0.02f%%RH\r\n", *temp, *humidity );

        } else {
            PRINTF( "Failed Temperature CRC\r\n" );
            hs.false_temp_count += 1;
        }
    } else {
        PRINTF( "Failed Humidity CRC\r\n" );
        hs.false_temp_count += 1;
    }
    return true;
}
/*
 * Calculate the CRC generator polynomial of x8 + x5 + x4 + 1, with an initialization of 0x00.
 */
uint8_t Si_crc( uint8_t *bytes, uint16_t length )
{

	uint8_t crc, i, j;

	crc = 0;

	for( i = 0; i < length; i++ ) {
		crc ^= bytes[ i ];
	    for( j = 8; j > 0; j-- ) {
	    	if( crc & 0x80 )
	    		crc = ( crc << 1 ) ^ 0x131;
	    	else
	    		crc = ( crc << 1 );
	    }

	}
	return crc;

}

uint8_t Si_crc8( uint8_t value, uint8_t seed )
{
    int i;

    for (i = 0; i < 8; i++) {

        if ((value ^ seed) & 0x80) {
//            seed  = ( seed <<= 1 ) ^ 131;// Caused Compiler Warning, so replaced <<= with <<.
        	seed = ( seed << 1 ) ^ 131;
        } else {
            seed <<= 1;
        }

        value <<= 1;
    }

    return seed;
}
