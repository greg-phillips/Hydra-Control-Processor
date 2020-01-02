/*
 * Copyright 2019, iMatrix Systems, Inc. All Rights Reserved.
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

/** @file led_color_defs.h
 *
 *  Created on: Novmember 05, 2019
 *      Author: greg.phillips
 *
 */

#ifndef LED_COLOR_DEFS_H_
#define LED_COLOR_DEFS_H_

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

const uint16_t orange_scan[ LEDS_NO_LEDS ] = {
                255,  140, 26,
};
const uint16_t green_blend_scan[ LEDS_NO_LEDS ] = {
                0,  255, 85,
};

const uint16_t blue_scan[ LEDS_NO_LEDS ] = {
                0,  0, 255,
};


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
 *               Function Definitions
 ******************************************************/

#endif /* LED_COLOR_DEFS_H_ */
