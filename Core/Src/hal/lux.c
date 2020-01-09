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
 *  lux.c
 *
 *  Created on: September, 2017
 *      Author: greg.phillips
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "structs.h"
#include "system.h"

#include "LTR_329ALS.h"
/******************************************************
 *                      Macros
 ******************************************************/
#ifdef PRINT_APOLLO_DEBUGS_FOR_HISTORY
    #undef PRINTF
    #define PRINTF(...) if( ( fc.log_messages & DEBUGS_FOR_HISTORY ) != 0x00 ) imx_cli_print(__VA_ARGS__)
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
/**
  * @brief set the mode of the sensor OFF - FALSE / ON - TRUE
  * Used when LED is On so as not to effect reading
  * @param  mode
  * @retval : None
  */
void lux_sensor_on( uint16_t mode )
{
    LTR_32ALS_set_mode( mode );
}
/**
  * @brief update lux read setting - this for routines that need instant values
  * @param  None
  * @retval : None
  */
bool update_lux(void)
{
    float lux_level, ir_level;

    if( LTR_329ALS_get_lux_level( &lux_level, &ir_level ) == true ) {
//        imx_printf( "Got LUX level: %f\r\n", lux_level );
        if( lux_level != 0 )  { // Ignore 0 reading - but count as an error - for review of performance
            /*
             * Save values
             */
            hs.current_lux_level = lux_level;
            hs.current_ir_lux_level = ir_level;
            hs.valid_lux_level = true;
            hs.valid_ir_lux_level = true;
        } else
            hs.false_lux_count += 1;
        return true;
    } else
        return false;
}
