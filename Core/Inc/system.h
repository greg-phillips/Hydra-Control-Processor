/*
 * Copyright 2020, iMatrix Systems, Inc. All Rights Reserved.
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
#ifndef SYSTEM_H_
#define SYSTEM_H_

/** @file .h
 *
 *  Created on: Janyary 02, 2020
 *      Author: greg.phillips
 *
 */
/*
 *	Defines for
 *
 */

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define LED_BAR_NO_LEDS                     ( 12 )      // Number of LEDS on our LED Ring
#define NO_DISPLAY_STATES                   ( 8 )

#define EV_TEMP_OFFSET                      ( 0.0 )     // Offset for device heating
#define SFLASH_DATA_STORAGE					( 4 * 1024 * 1024 )
/******************************************************
*               Function Definitions
******************************************************/


/******************************************************
 *                   Enumerations
 ******************************************************/
enum {
    EV_LED_GREEN,
    EV_LED_RED,
    EV_LED_BLUE,
    LEDS_NO_LEDS,
};

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

#endif /* SYSTEM_H_ */
