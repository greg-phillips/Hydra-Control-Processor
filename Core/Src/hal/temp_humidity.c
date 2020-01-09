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
 *  temp_humidity.c
 *
 *  Created on: September, 2017
 *      Author: greg.phillips
 *
 */

#include <stdint.h>
#include <stdio.h>

#include "structs.h"
#include "system.h"

#include "Si7020.h"

#ifdef PRINT_APOLLO_DEBUGS_FOR_ALL
    #undef PRINTF
    #define PRINTF(...) if( ( evc.log_messages & DEBUG_GENERAL ) != 0x00 ) imx_cli_print( __VA_ARGS__)
#elif !defined PRINTF
    #define PRINTF(...)
#endif

/******************************************************
 *                      Macros
 ******************************************************/

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
  * @brief update temp & humidity
  * @param  None
  * @retval : success / failure
  */

bool update_temp_humidity(void)
{
    float temperature, humidity;
    if( Si7020_get_temp( &temperature, &humidity ) == true ) {
        hs.current_temp_level = temperature;
        hs.current_humidity_level = humidity;
        hs.valid_temp_level = true;
        hs.valid_humidity_level = true;
        return true;
    } else
        return false;
}
