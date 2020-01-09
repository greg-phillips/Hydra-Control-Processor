/*
 * $Copyright 2013-2017, Sierra Telecom, Inc.
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Sierra Telecom, Inc.;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Sierra Telecom, Inc.$
 */

/** @file sensors.c
 *
 *  Created on: Feb 2, 2017
 *      Author: greg.phillips
 */

#include <stdint.h>
#include <stdbool.h>
#include <memory.h>

#include "common.h"
#include "structs.h"
#include "sensors.h"
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

extern imx_control_sensor_block_t imx_sensors_defaults[];
extern cp_control_sensor_block_t sb[];
/******************************************************
 *               Function Definitions
 ******************************************************/
void load_config_defaults_generic_scb( uint16_t arg )
{
	memcpy( &sb[ arg ].csb, &imx_sensors_defaults[ arg ], sizeof( imx_control_sensor_block_t ) );
}
