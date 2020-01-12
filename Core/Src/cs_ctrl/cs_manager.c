/*
 * Copyright 2020, iMatrix Systems, Inc.. All Rights Reserved.
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

/** @file cs_manager.c
 *
 *	Manage the various integrated controls sensors that are connected the to Control Processor
 *
 *  Created on: January 8, 2020
 *      Author: greg.phillips
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <memory.h>

#include "system.h"
#include "common.h"
#include "structs.h"
#include "memory_manager.h"
#include "product.h"

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
cp_control_sensor_block_t cb[ IMX_NO_CONTROLS ];
cp_control_sensor_block_t sb[ IMX_NO_SENSORS ];

extern imx_control_sensor_block_t imx_controls_defaults[];
extern imx_control_sensor_block_t imx_sensors_defaults[];
extern hydra_status_t hs;
/******************************************************
 *               Function Definitions
 ******************************************************/
/**
  * @brief
  * @param  None
  * @retval : None
  */
void cs_init(void)
{
	int16_t sector;
	uint16_t i;
	/*
	 * Start with clean data
	 */
	memset( &cb, 0x00, sizeof( cb ) );
	memset( &sb, 0x00, sizeof( sb ) );
	/*
	 * Use the values in FLASH to initialize the initial settings	 *
	 */
	for( i = 0; i < hs.no_controls; i++ ) {
		memcpy( &cb[ i ].csb, &imx_controls_defaults[ i ], sizeof( imx_control_sensor_block_t) );
	}
	for( i = 0; i < hs.no_controls; i++ ) {
		memcpy( &sb[ i ].csb, &imx_sensors_defaults[ i ], sizeof( imx_control_sensor_block_t) );
	}
	/*
	 * Allocate initial FLASH sectors for each of these items
	 */
	for( i = 0; i < hs.no_controls; i++ ) {
		sector = get_next_sector();
		if( sector == -1 ) {
			/*
			 * This is broken - needs to be fixed before continuing. THIS IS A BUG
			 */
			return;
		}
		init_crc_value = 0xFFFFFFFF;
		cb[ i ].start_sector = sector;
		cb[ i ].start_offset = 0;
		cb[ i ].end_sector = sector;
		cb[ i ].end_offset = 0;
		cb[ i ].count = 0;
	}
	for( i = 0; i < hs.no_sensors; i++ ) {
		sector = get_next_sector();
		if( sector == -1 ) {
			/*
			 * This is broken - needs to be fixed before continuing. THIS IS A BUG
			 */
			return;
		}
		sb[ i ].start_sector = sector;
		sb[ i ].start_offset = 0;
		sb[ i ].end_sector = sector;
		sb[ i ].end_offset = 0;
		sb[ i ].count = 0;
	}
}
