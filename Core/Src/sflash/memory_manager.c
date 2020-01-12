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

/** @file memory_manager.c
 *
 *  Created on: January 10, 2020
 *      Author: greg.phillips
 *
 *	The functions in this file will handle the management of the SFLASH
 *	storage area used by the data collection system
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "structs.h"
#include "spi_flash.h"
#include "memory_manager.h"
/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define	NO_TSD_ENTRIES_PER_SECTOR	( 4096 - ( 4096 / 4 ) - 8 )		// 4096 Bytes in sector - each record 4 bytes - 8 Bytes for sector linking data
#define	NO_EVT_ENTRIES_PER_SECTOR	( 4096 - ( 4096 / 8 ) - 8 )		// 4096 Bytes in sector - each record 8 bytes ( Time stamp and Data )- 8 Bytes for sector linking data
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
sector_assignment_table_t sat;
cp_control_sensor_block_t cb[ IMX_NO_CONTROLS ];
cp_control_sensor_block_t sb[ IMX_NO_SENSORS ];

/******************************************************
 *               Function Definitions
 ******************************************************/
/**
  * @brief	Save a TSD entry to SFLASH
  * check if there is space in the current sector, if not allocate a new sector and add index, CRC to the last 8 bytes
  * @param  type of device Control / Sensor, entry number in array, value
  * @retval : None
  */
void save_tsd( imx_peripheral_type_t type, unit16_t entry, uint32_t value )
{
	uint16_t new_sector;
	control_sensor_data_t *csd;
	imx_control_sensor_block_t *csb;
	bool sample_driven, allocate_new_sector;

	allocate_new_sector = false;
	sample_driven = false;
    if( type == IMX_CONTROLS ) {
    	csb = &cb[ 0 ].csb;
    	csd = &cd[ 0 ].csd;
        if( csb[ entry ].sample_rate != 0 ) {
        	sample_driven = true;
        	if( cb[ entry ].count >= NO_TSD_ENTRIES_PER_SECTOR )
        		allocate_new_sector = true;
        } else {
        	if( cb[ entry ].count >= NO_EVT_ENTRIES_PER_SECTOR )
        		allocate_new_sector = true;
        }
    } else {
    	csb = &sb[ 0 ].csb;
    	csd = &sb[ 0 ].csd;
        if( csb[ entry ].sample_rate != 0 ) {
        	sample_driven = true;
        	if( sb[ entry ].count >= NO_TSD_ENTRIES_PER_SECTOR )
        		allocate_new_sector = true;
        } else {
        	if( sb[ entry ].count >= NO_EVT_ENTRIES_PER_SECTOR )
        		allocate_new_sector = true;
        }
    }

    if( allocate_new_sector == true ) {
    	/*
    	 * Calculate the CRC
    	 */

    	new_sector = get_next_sector();
}
/**
  * @brief	Initialize SFLASH data collection area
  * The Sector Assignment Table (A section RAM that maps to sectors in the SFLASH) is used to
  * show which sectors in the SFLASH are currently allocated. Sectors are 4K in size, SFLASH_DATA_STORAGE of storage
  * this works out to SAT_NO_SECTORS 32 bit entries
  * @param  None
  * @retval : None
  */
void init_sat(void)
{
	uint16_t i;
	/*
	 * On power up the SAT is initialized to 0
	 */
	memset( (void*) &sat, 0x00, sizeof( sector_assignment_table_t ) );
	/*
	 * Erase all sections of the SFLASH
	 */
	printf( "Erasing SAT ..." );
	for( i = 0; i < SAT_NO_SECTORS; i++ ) {
		erase_sector( i );
	}
	printf( "SAT Erased\r\n" );
}
/**
  * @brief	Get the next available sector in the SFLASH for data storage
  * @param  None
  * @retval : None
  */
int16_t get_next_sector(void)
{
	uint16_t i, j, sector;
	uint32_t mask;
	/*
	 * Scan through the SAT to find the first available entry
	 */
	j = 0;
	for( i = 0; i < SAT_NO_SECTORS; i++ ) {
		for( mask = 0x01; mask; mask = mask << 1 ) {
			if( ( sat.block[ i ] & mask ) == 0x00 ) {
				/*
				 * Found available entry, calculate number
				 */
				sector = ( i * 32 ) + j;
				return sector;
			}
			j += 1;
		}
	}
	/*
	 * Add recovery code here to drop oldest entry - later
	 */
	printf( "SFLASH Data storage area exhausted\r\n" );
	return -1;
}

/**
  * @brief	Free sector
  * Erase sector in flash and clear bit in SAT
  * @param  sector
  * @retval : None
  */
void free_sector( uint16_t sector )
{
	uint16_t sat_entry;
	uint32_t bit_mask;

	if( sector >= SAT_NO_SECTORS ) {
		/*
		 * Flag as BUG
		 */
		printf( "Attempting to release invalid sector: %u\r\n", sector );
		return;
	}
	erase_sector( sector );
	sat_entry = ( sector & 0xFFE0 ) >> 4;
	bit_mask = 1 << ( sector & 0x1F );
	sat.block[ sat_entry ] &= ~bit_mask;
}
