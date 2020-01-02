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

/** @file ck_time.c
 *
 *  Created on: January 1, 2020
 *      Author: greg.phillips
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>


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

/******************************************************
 *               Function Definitions
 ******************************************************/
/**
  * @brief 	Check to see if the time2 is latear than time1
  * @param  None
  * @retval : None
  */
/**
 * Taking into consideration the possibility of a rollover in the tick time every 47 days,
 * return TRUE if the first time parameter is later than the last.
 * Otherwise return FALSE.
 * NOTE: Assume that the two times are no more than a quarter of the possible range of values apart.
 *
 * written by Eric Thelin 29 June 2016
 */
bool imx_is_later( uint32_t time1, uint32_t time2 )
{
	const uint32_t range_top_quarter = 0xC0000000;
	const uint32_t range_mid_point =   0x80000000;
	const uint32_t range_low_quarter = 0x40000000;

	if ( time1 >= range_mid_point ) {
		if ( time2 >= range_low_quarter ) {
			return ( time1 > time2 );
		}
		else {// Assume that time2 has rolled over and is really the bigger number
			return false;
		}
	}
	else {// time1 is in the low half of the range

		if ( time2 <= range_top_quarter ) {
			return ( time1 > time2 );
		}
		else {// Assume time1 has rolled over and is really the bigger number
			return true;
		}
	}
}
