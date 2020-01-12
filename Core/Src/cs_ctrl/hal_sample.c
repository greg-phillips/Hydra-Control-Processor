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
 * If no EULA applies, Sierra hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Sierra's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Sierra.
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
/** @file hal_sample.c
 *
 *  Created on: Feb 2, 2017
 *      Author: greg.phillips
 */




#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "structs.h"
#include "ck_time.h"
#include "memory_manager.h"
#include "product.h"
#include "hal_sample.h"
/******************************************************
 *                      Macros
 ******************************************************/

#ifdef PRINT_DEBUGS_FOR_SAMPLING
    #undef PRINTF
    #define PRINTF(...) if( ( device_config.log_messages & DEBUGS_FOR_SAMPLING ) != 0x00 ) imx_cli_print(__VA_ARGS__)
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
typedef struct _sample_status {
    uint16_t active_ble_device;
    uint16_t no_controls, no_sensors;
    unsigned int end_of_controls            : 1;
    unsigned int end_of_sensors             : 1;
    unsigned int skip_sampling              : 1;
} sample_status_t;    // This will be zeroed on start up

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
bool check_int_percent( int32_t current_value, int32_t last_value, uint16_t percentage );
bool check_uint_percent( uint32_t current_value, uint32_t last_value, uint16_t percentage );
bool check_float_percent( float current_value, float last_value, uint16_t percentage );

/******************************************************
 *               Variable Definitions
 ******************************************************/
sample_status_t sample_status;
uint16_t active_sensor;
uint16_t active_control;
extern hydra_status_t hs;
cp_control_sensor_block_t cb[ IMX_NO_CONTROLS ];
cp_control_sensor_block_t sb[ IMX_NO_SENSORS ];
extern imx_functions_t imx_control_functions[], imx_sensor_functions[];
/******************************************************
 *               Function Definitions
 ******************************************************/
/**
  * @brief Initialize the sampling process
  *
  * Start with product controls and sensors
  *
  * if BLE Devices are enabled allow switching to them after processing products
  * This routine is also called when BLE Devices are dropped from the system
  *
  * @param  None
  * @retval : None
  */
void hal_sample_init(void)
{
    sample_status.active_ble_device = 0;
    sample_status.no_controls = hs.no_controls;
    sample_status.no_sensors = hs.no_controls;
    sample_status.end_of_controls = false;
    sample_status.end_of_sensors = false;
    sample_status.skip_sampling = false;

    active_control = 0;
    active_sensor = 0;
}
/**
  * @brief process a the sampling of controls and sensors
  * @param  None
  * @retval : None
  */
void hal_sample( imx_peripheral_type_t type, uint32_t current_time )
{
	uint16_t i, *active;
	uint8_t status;
	bool percent_change_detected;
    imx_data_32_t sampled_value;
	control_sensor_data_t *csd;
	imx_control_sensor_block_t *csb;
	imx_functions_t *f;

	/*
	 * To prevent complier warnings
	 */
	active = &active_control;
	f = NULL;

	if( type == IMX_CONTROLS ) {
		if( sample_status.no_controls == 0 ) {
		    sample_status.end_of_controls = true;
		} else {
		    active = &active_control;
	        if( *active >= sample_status.no_controls ) {
	            *active = 0;
	            sample_status.end_of_controls = true;
	        }
        }
	} else {
        if( sample_status.no_sensors == 0 ) {
            sample_status.end_of_sensors = true;
        } else {
            active = &active_sensor;
            if( *active >= sample_status.no_sensors ) {
                *active = 0;
                sample_status.end_of_sensors = true;
            }
        }
	}

    /*
     * Set Control and Data Pointers
     */
    /* if( sample_status.upload_source == IMX_UPLOAD_GATEWAY ) Always fall back to GATEWAY */
    if( type == IMX_CONTROLS ) {
        if( ( sample_status.no_controls == 0 ) || ( sample_status.end_of_controls == true ) )
            return;
        else {
            csb = &cb[ 0 ].csb;
            csd = &cb[ 0 ].csd;
            f = &imx_control_functions[ 0 ];
        }
    } else {
        if( ( sample_status.no_sensors == 0 ) || ( sample_status.end_of_sensors == true ) )
            return;
        else {
            csb = &sb[ 0 ].csb;
            csd = &sb[ 0 ].csd;
            f = &imx_sensor_functions[ 0 ];
        }
    }


#ifdef PRINT_DEBUGS_FOR_SAMPLING
	if( device_config.log_messages & DEBUGS_FOR_SAMPLING )
	    wiced_rtos_delay_milliseconds( 100 ); // Used for debug to slow things down
#endif
	/*
	 * Check Control / Sensor, update this sensor stored data if it changes warning level or sample rate is due
	 *
	 * Sample rate of 0 represents event driven
	 */
	if( ( csb[ *active ].enabled == true ) && ( csb[ *active ].sample_rate > 0 ) && ( imx_is_later( current_time, csd[ *active ].last_poll_time + csb[ *active ].poll_rate ))) {
		status = 0;	// Controls may not have an update function as the may just be set remotely
		if( sample_status.skip_sampling == false) {
	        if( f[ *active ].update != NULL ) {
	            status = ( f[ *active ].update)( f[ *active ].arg, &sampled_value );
	#ifdef PRINT_DEBUGS_FOR_SAMPLING
	            if( device_config.log_messages & DEBUGS_FOR_SAMPLING ) {
	                imx_cli_print( "Sampled %s: %u, result: %u", ( type == IMX_CONTROLS ) ? "Control" : "Sensor", *active, status );
	                imx_cli_print( ", Value: " );
	                switch( csb[ *active ].data_type ) {
	                    case IMX_INT32 :
	                        imx_cli_print( "%ld", sampled_value.int_32bit );
	                        break;
	                    case IMX_FLOAT :
	                        imx_cli_print( "%f", sampled_value.float_32bit );
	                        break;
	                    case IMX_UINT32 :
	                    default :
	                        imx_cli_print( "%lu", sampled_value.uint_32bit );
	                        break;
	                }
	                imx_cli_print( "\r\n" );
	            }
	#endif
	            if( status == IMX_SUCCESS ) {
	                csd[ *active ].last_poll_time = current_time;                           // Got valid data this time
	                csd[ *active ].last_raw_value.uint_32bit = sampled_value.uint_32bit;    // Its all just 32 bit data
	                csd[ *active ].valid = true;        // We have a sample
	                csd[ *active ].error = status;      // Reset for correction
	            } else if( status == IMX_NO_DATA )
	                ;   // Do nothing - keep using existing data - waiting for control/sensor to finish acquisition
	            else {
	                PRINTF( "Error Reading sensor: %u - %u\r\n", *active, status );
	                csd[ *active ].errors += 1;
	                csd[ *active ].error = status;
	                /*
	                 * see if change in error or all we are getting is errors. - Only send once per batch
	                 */
	                if( ( csd[ *active ].error != csd[ *active ].last_error ) ||
	                    ( imx_is_later( current_time, csd[ *active ].last_sample_time + ( (uint32_t) csb[ *active ].sample_batch_size * 1000L  ) ) == true ) ) {
	    /*
	     *
	                    print_status( "Error: %u, Last Error: %u, current_time: %lu, time difference: %lu\r\n", csd[ *active ].error, csd[ *active ].last_error, csd[ *active ].last_sample_time,
	                        ( csd[ *active ].last_sample_time + (wiced_time_t) ( (uint32_t) csb[ *active ].sample_batch_size * 1000L  ) - (uint32_t) current_ms_time )  );
	    */
	                    csd[ *active ].last_sample_time = current_time;
	                    csd[ *active ].last_error = csd[ *active ].error;
	                    csd[ *active ].send_on_error = true;
	                }
	            }
	        }
		} else {
		    /*
		     * This sensor is updated independently - BLE scan data etc.
		     *
		     * Check if we have missed 10 poll attempts. - Flag the device in error if this is the case
		     *
		     */
		    if(imx_is_later( current_time, csd[ *active ].last_poll_time + ( csb[ *active ].poll_rate ) * 10 ) ) {
                PRINTF( "Error Reading sensor: %u - %u\r\n", *active, status );
                csd[ *active ].errors += 1;
                csd[ *active ].error = IMX_NO_BLE_RESPONSE;
                /*
                 * see if change in error or all we are getting is errors. - Only send once per batch
                 */
                if( ( csd[ *active ].error != csd[ *active ].last_error ) ||
                    ( imx_is_later( current_time, csd[ *active ].last_sample_time + ( (uint32_t) csb[ *active ].sample_batch_size * 1000L  ) ) == true ) ) {
    /*
     *
                    print_status( "Error: %u, Last Error: %u, current_time: %lu, time difference: %lu\r\n", csd[ *active ].error, csd[ *active ].last_error, csd[ *active ].last_sample_time,
                        ( csd[ *active ].last_sample_time + (wiced_time_t) ( (uint32_t) csb[ *active ].sample_batch_size * 1000L  ) - (uint32_t) current_ms_time )  );
    */
                    csd[ *active ].last_sample_time = current_time;
                    csd[ *active ].last_error = csd[ *active ].error;
                    csd[ *active ].send_on_error = true;
                }
		    }
		}
        /*
         * Update value with any calibration settings
         */
        if( csb[ *active ].calibration_value.uint_32bit != 0 ) {
            /*
             * Make Adjustment based on type
             */
            if( csb[ *active ].data_type == IMX_INT32 )
                csd[ *active ].last_value.int_32bit = csd[ *active ].last_raw_value.int_32bit + csb[ *active ].calibration_value.int_32bit;
            else if( csb[ *active ].data_type == IMX_FLOAT )
                csd[ *active ].last_value.float_32bit = csd[ *active ].last_raw_value.float_32bit + csb[ *active ].calibration_value.float_32bit;
            else {
                /*
                 * Make sure value does not go negative
                 */
                if( ( (int32_t) ( csd[ *active ].last_value.uint_32bit ) + csb[ *active ].calibration_value.int_32bit ) < 0 )
                    csd[ *active ].last_value.uint_32bit = 0;
                else
                    csd[ *active ].last_value.int_32bit += csb[ *active ].calibration_value.int_32bit;
            }
        } else {
            csd[ *active ].last_value.int_32bit = csd[ *active ].last_raw_value.int_32bit;
        }

		/*
		 * Process current entry with value new or old
		 */
        /*
         * Check if the data is in warning levels for the sensor
         */
        csd[ *active ].warning = IMX_INFORMATIONAL;  // Assume for now
        /*
         * Each time thru the loop will check for the next most severe level and set the highest by the end
         */
        for( i = IMX_WATCH; i < IMX_WARNING_LEVELS; i++ ) {
            /*
             * Do we check if below current level
             */
            if( ( csb[ *active ].use_warning_level_low & ( 0x1 << ( i - IMX_WATCH ) ) ) != 0 )
                switch( csb[ *active ].data_type ) {
                    case IMX_INT32 :
                        if( csd[ *active ].last_value.int_32bit < csb[ *active ].warning_level_low[ i - IMX_WATCH ].int_32bit )
                            csd[ *active ].warning = i;  // Now set to this level
                        break;
                    case IMX_FLOAT :
                        if( csd[ *active ].last_value.float_32bit < csb[ *active ].warning_level_low[ i - IMX_WATCH ].float_32bit )
                            csd[ *active ].warning = i;  // Now set to this level
                        break;
                    case IMX_UINT32 :
                    default :
                        if( csd[ *active ].last_value.uint_32bit < csb[ *active ].warning_level_low[ i - IMX_WATCH ].uint_32bit )
                            csd[ *active ].warning = i;  // Now set to this level
                        break;
                }
            /*
             * Do we check if above current level
             */
            if( ( csb[ *active ].use_warning_level_high & ( 0x1 << ( i - IMX_WATCH ) ) ) != 0 )
                switch( csb[ *active ].data_type ) {
                    case IMX_INT32 :
                        if( csd[ *active ].last_value.int_32bit > csb[ *active ].warning_level_low[ i - IMX_WATCH ].int_32bit )
                            csd[ *active ].warning = i;  // Now set to this level
                        break;
                    case IMX_FLOAT :
                        if( csd[ *active ].last_value.float_32bit > csb[ *active ].warning_level_low[ i - IMX_WATCH ].float_32bit )
                            csd[ *active ].warning = i;  // Now set to this level
                        break;
                    case IMX_UINT32 :
                    default :
                        if( csd[ *active ].last_value.uint_32bit > csb[ *active ].warning_level_low[ i - IMX_WATCH ].uint_32bit )
                            csd[ *active ].warning = i;  // Now set to this level
                        break;
                }
        }
        /*
         * Do we check percent change
         */
        percent_change_detected = false;
        if( csb[ *active ].send_on_percent_change == true )
            switch( csb[ *active ].data_type ) {
                case IMX_INT32 :
                    if( check_int_percent( csd[ *active ].last_value.int_32bit, csd[ *active ].last_value.int_32bit, csb[ *active ].percent_change_to_send ) )
                        percent_change_detected = true;
                    break;
                case IMX_FLOAT :
                    if( check_float_percent( csd[ *active ].last_value.float_32bit, csd[ *active ].last_value.float_32bit, csb[ *active ].percent_change_to_send ) )
                        percent_change_detected = true;
                    break;
                case IMX_UINT32 :
                default :
                    if( check_uint_percent( csd[ *active ].last_value.uint_32bit, csd[ *active ].last_value.uint_32bit, csb[ *active ].percent_change_to_send ) )
                        percent_change_detected = true;
                    break;
            }

        /*
         * See if we save this entry
         *
         * The criteria is based on the following
         *
         * is this sample on the sample rate so save as standard
         * is this sample a change in warning level
         * is this sample a change of >= Change percentage level - if enabled
         *
         */
        if( ( ( csb[ *active ].send_imatrix == true ) && ( csd[ *active ].valid == true ) ) &&
            ( ( imx_is_later( current_time, csd[ *active ].last_sample_time + ( csb[ *active ].sample_rate ) ) == true ) ||
            ( csd[ *active ].warning != csd[ *active ].last_warning ) ||
            ( percent_change_detected == true ) ||
            ( csd[ *active ].send_on_error == true ) ) ) {
        	/*
        	 * Add to the SFLASH Data store
        	 */
        	save_tsd( type, *active, csd[ *active ].last_value.uint_32bit ); // Save this entry its all just 32 bit data
/*
            imx_printf( "Saving %s value for sensor(%u): %s, Saved entries: %u\r\n", type == IMX_CONTROLS ? "Control" : "Sensor", *active, csb[ *active ].name, ( csd[ *active ].no_samples + 1 ) );
*/
            csd[ *active ].no_samples += 1;
            csd[ *active ].last_sample_time = current_time;
            /*
             * See if the we need to wake up the Communications Processor to send
             */
            if( ( csd[ *active ].warning != csd[ *active ].last_warning ) ||
                ( csd[ *active ].update_now == true ) ||
                ( percent_change_detected == true ) ) {
#ifdef PRINT_DEBUGS_FOR_SAMPLING
                if( device_config.log_messages & DEBUGS_FOR_SAMPLING ) {
                    imx_cli_print( "Setting %s: %u, ID: 0x%08lx to send batch of: %u, batch size %u, sample_now: %s sensor_warning: %u, last: %u, %%change detected: %s\r\n", type == IMX_CONTROLS ? "Control" : "Sensor",
                            *active, csb[ *active ].id, csd[ *active ].no_samples, csb[ *active ].sample_batch_size, csd[ *active ].update_now ? "true" : "false",
                            csd[ *active ].warning, csd[ *active ].last_warning, percent_change_detected ? "true" : "false" );
                }
#endif
                csd[ *active ].update_now = false;
                csd[ *active ].last_warning = csd[ *active ].warning;
                csd[ *active ].send_batch = true;    // Send this now
                hs.send_data_records_now = true;
            }
        }
	}

	*active += 1;	// Process next entry
}

/**
  * @brief	Check if the new value exceeds the previous by great than a defined percentage
  * 		Three routines, one for each type of data we support
  * @param  current value, last value, percentage
  * @retval : bool
  */
bool check_int_percent( int32_t current_value, int32_t last_value, uint16_t percentage )
{
	current_value = abs( current_value );
	last_value = abs( last_value );
//	print_status( "Comparing %ld against %ld (high) %ld (low)\r\n", ( current_value * 100 ), ( last_value * ( 100 + percentage ) ), ( last_value * ( 100 - percentage ) ) );

	if( ( current_value * 100 ) > ( last_value * ( 100 + percentage ) ) )
		return true;
	if( ( current_value * 100 ) < ( last_value * ( 100 - percentage ) ) )
		return true;
//	print_status( "With in range\r\n" );
	return false;
}
bool check_uint_percent( uint32_t current_value, uint32_t last_value, uint16_t percentage )
{
	if( ( current_value * 100 ) > ( last_value * ( 100 + percentage ) ) )
		return true;
	if( ( current_value * 100 ) < ( last_value * ( 100 - percentage ) ) )
		return true;
	return false;
}
bool check_float_percent( float current_value, float last_value, uint16_t percentage )
{
	current_value = (float) fabs( (double) current_value );
	last_value = (float) fabs( (double) last_value );
//	print_status( "Comparing %f against %f (high) %f (low)\r\n", ( current_value ), ( last_value * ( 1 + (float)percentage/100 ) ), ( last_value * ( 1 - (float)percentage/100 ) ) );
	if( ( current_value ) > ( last_value * ( 1 + ( (float)percentage/100 ) ) ) )
		return true;
	if( ( current_value ) < ( last_value * ( 1 - ( (float)percentage/100 ) ) ) )
		return true;
//	print_status( "With in range\r\n" );
	return false;
}

