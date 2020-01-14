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
#include <memory.h>

#include "main.h"
#include "system.h"
#include "structs.h"
#include "product.h"
#include "spi_flash.h"
#include "utc_time.h"
#include "memory_manager.h"
/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define SECTOR_OVERHEAD				( 10 )
#define SFLASH_SECTOR_SIZE			( 4096 )
#define MAX_SECTOR_DATA_SIZE		( SFLASH_SECTOR_SIZE - SECTOR_OVERHEAD )
#define START_OF_DATA				( 0x400000 )
#define TSD_RECORD_SIZE				( 4 )
#define EVT_RECORD_SIZE				( 8 )
#define	NO_TSD_ENTRIES_PER_SECTOR	( ( SFLASH_SECTOR_SIZE - SECTOR_OVERHEAD ) / TSD_RECORD_SIZE )		// SFLASH_SECTOR_SIZE Bytes in sector - each record 4 bytes - 8 Bytes for sector linking data
#define	NO_EVT_ENTRIES_PER_SECTOR	( ( SFLASH_SECTOR_SIZE - SECTOR_OVERHEAD ) / EVT_RECORD_SIZE )		// SFLASH_SECTOR_SIZE Bytes in sector - each record 8 bytes ( Time stamp and Data )- 8 Bytes for sector linking data
#define SECTOR_ID_ADDRESS			( 0xFF4 )
#define NEXT_SECTOR_ADDRESS			( 0xFFA )
#define CRC_ADDRESS					( 0xFFC )
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
static void IMX_CRC_Init( uint32_t initial_value );
/******************************************************
 *               Variable Definitions
 ******************************************************/
sector_assignment_table_t sat;
cp_control_sensor_block_t cb[ IMX_NO_CONTROLS ];
cp_control_sensor_block_t sb[ IMX_NO_SENSORS ];
extern CRC_HandleTypeDef hcrc;
extern hydra_status_t hs;
/******************************************************
 *               Function Definitions
 ******************************************************/

/**
  * @brief	Save a TSD or Event Driven entry to SFLASH
  * check if there is space in the current sector, if not allocate a new sector and add index, CRC to the last 10 bytes
  * @param  type of device Control / Sensor, entry number in array, value
  * @retval : None
  */
void save_tsd_evt( imx_peripheral_type_t type, uint16_t entry, uint32_t value )
{
	uint16_t new_sector, *end_sector, *count, var_data_length;
	uint32_t *init_crc_value, byte_offset, utc_time;
	cp_var_data_entry_t *var_data;
	imx_control_sensor_block_t *csb;
	bool event_driven, allocate_new_sector;

	allocate_new_sector = false;
	event_driven = false;
	var_data = 0x00;		// To prevent compiler warning - this is initialized below if this is var data
	/*
	 * Event Driven saves Time Stamp & Value pair
	 */
	if( hs.time_set_with_NTP == true ) {
		utc_time = get_current_utc();
	} else
		utc_time = 0;        // Tell iMatrix to assign

    if( type == IMX_CONTROLS ) {
    	csb = &cb[ 0 ].csb;
    	init_crc_value = &cb[ entry ].data.init_crc_value;
		end_sector = &cb[ entry ].data.end_sector;
		count = &cb[ entry ].data.count;
    } else {
    	csb = &sb[ 0 ].csb;
    	init_crc_value = &sb[ entry ].data.init_crc_value;
		end_sector = &sb[ entry ].data.end_sector;
		count = &sb[ entry ].data.count;
    }
    if( csb[ entry ].data_type == IMX_VARIABLE_LENGTH ) {
    	/*
    	 * For variable length records the count is actually the BYTE count not the number of records
    	 */
    	var_data = (cp_var_data_entry_t *) value;
    	var_data_length = var_data->length;
    	if( ( *count + var_data_length + 2 ) >= MAX_SECTOR_DATA_SIZE )
			allocate_new_sector = true;
    } else {
    	if( csb[ entry ].sample_rate != 0 ) {
    		if( *count >= NO_TSD_ENTRIES_PER_SECTOR )
    			allocate_new_sector = true;
    	} else {
    		event_driven = true;
    		if( *count >= NO_EVT_ENTRIES_PER_SECTOR )
    			allocate_new_sector = true;
    	}
    }
    /*
     * Initialize the CRC controller
     */
    IMX_CRC_Init( *init_crc_value );
    /*
     * Calculate the byte offset in the FLASH based on the sector we are using
     */
	byte_offset = START_OF_DATA + ( *end_sector * SFLASH_SECTOR_SIZE );

    if( allocate_new_sector == true ) {
    	new_sector = get_next_sector();
    	/*
    	 * Save device ID, new sector info, and CRC
    	 */
    	*init_crc_value = HAL_CRC_Accumulate(&hcrc, (uint32_t *)&csb[ entry ].id, 4 );
    	sFLASH_WriteBuffer( (uint8_t *) &csb[ entry ].id, byte_offset + SECTOR_ID_ADDRESS, 4 );

    	*init_crc_value = HAL_CRC_Accumulate(&hcrc, (uint32_t *)&new_sector, 2 );
    	sFLASH_WriteBuffer( (uint8_t *) &init_crc_value, byte_offset + NEXT_SECTOR_ADDRESS, 2 );

    	sFLASH_WriteBuffer( (uint8_t *) &csb[ entry ].id, byte_offset + CRC_ADDRESS, 4 );

    	*init_crc_value = 0xFFFFFFFF;
        /*
         * Initialize the CRC controller
         */
        IMX_CRC_Init( *init_crc_value );
        *end_sector = new_sector;
        /*
         * re-Calculate the byte offset in the FLASH based on the sector we are using
         */
    	byte_offset = START_OF_DATA + ( *end_sector * SFLASH_SECTOR_SIZE );
    	/*
    	 * Reset the start point in the sector
    	 */
    	*count = 0;
    }
    /*
     * Save the value to SFLASH
     */
    byte_offset += *count * EVT_RECORD_SIZE;

    if( event_driven == true ) {
    	/*
    	 * Write out the time first.
    	 * Add to CRC
    	 */
    	*init_crc_value = HAL_CRC_Accumulate(&hcrc, (uint32_t *)&utc_time, 4 );
    	sFLASH_WriteBuffer( (uint8_t *) &csb[ entry ].id, byte_offset, 4 );
    	/*
    	 * Save Value
    	 */
    	if( csb[ entry ].data_type == IMX_VARIABLE_LENGTH ) {
    		*init_crc_value = HAL_CRC_Accumulate(&hcrc, (uint32_t *)&var_data_length, 2 );
    		sFLASH_WriteBuffer( (uint8_t *) &var_data_length, byte_offset + 4, 2 );
    		*init_crc_value = HAL_CRC_Accumulate(&hcrc, (uint32_t *)&var_data->data, var_data_length );
    		sFLASH_WriteBuffer( (uint8_t *) &var_data->data, byte_offset + 6, var_data_length );
    	    *count += var_data_length + 2;
    	} else {
    		*init_crc_value = HAL_CRC_Accumulate(&hcrc, (uint32_t *)&value, 4 );
    		sFLASH_WriteBuffer( (uint8_t *) &value, byte_offset + 4, 4 );
    	    *count += 1;
    	}
    } else {
    	/*
    	 * Save Value
    	 */
    	*init_crc_value = HAL_CRC_Accumulate(&hcrc, (uint32_t *)&value, 4 );
    	sFLASH_WriteBuffer( (uint8_t *) &value, byte_offset, 4 );
        *count += 1;
    }

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
	hs.erase_cycles += 1;
	sat_entry = ( sector & 0xFFE0 ) >> 4;
	bit_mask = 1 << ( sector & 0x1F );
	sat.block[ sat_entry ] &= ~bit_mask;
}
/**
  * @brief	Initialize CRC Unit with value
  * @param  Initial Value
  * @retval : None
  */
static void IMX_CRC_Init( uint32_t initial_value )
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.InitValue = initial_value;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK) {
    printf( "Failed to initialize CRC Device\r\n" );
  }
}
