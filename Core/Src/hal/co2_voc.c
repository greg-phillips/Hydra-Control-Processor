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
 *  co2_voc.c
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
#include "sgp30_interface.h"
#include "co2_voc.h"
/******************************************************
 *                      Macros
 ******************************************************/
#ifdef PRINT_DEBUGS_FOR_SENSORS
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
  * @brief update CO2 & VOC setting - this for routines that need instant values
  * @param  None
  * @retval : True / False
  */
bool update_co2_voc(void)
{
    uint16_t tvoc_ppb, co2_eq_ppm;
    if( get_voc_co2_eq( &tvoc_ppb, &co2_eq_ppm ) == true ) {
        hs.current_tvoc_level = tvoc_ppb;
        hs.valid_tvoc_level = true;
        hs.current_co2_level = co2_eq_ppm;
        hs.valid_co2_level = true;
        return true;
    } else {
        return false;
    }
}
