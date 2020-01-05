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
#ifndef LEDS_H_
#define LEDS_H_

/** @file leds.h
 *
 *  Created on: Janyary 01, 2020
 *      Author: greg.phillips
 *
 */
/*
 *	Defines for LED Display System
 *
 */

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
 *               Function Definitions
 ******************************************************/
void led_init(void);
void process_led_display( uint32_t current_time );
void update_led_bar(void);
void set_led_bar( uint8_t led, uint16_t red, uint16_t green, uint16_t blue );
void set_led_bar_raw( uint8_t led, uint16_t red_level, uint16_t green_level, uint16_t blue_level );
void init_led_bar(void);
void led_bar_off(void);
void led_bar_scan_setup( uint16_t red, uint16_t green, uint16_t blue, uint16_t transision_period );
void led_bar_breath_setup( uint16_t red, uint16_t green, uint16_t blue, uint16_t transision_period );
void led_bar_wifi_setup_mode(void);
void led_bar_scan_mode_right( uint32_t current_time );
void led_bar_scan_mode_left( uint32_t current_time );
void led_bar_scan_mode_dual( uint32_t current_time );
void led_bar_breath_mode( uint32_t current_time );
void update_led_bar(void);
void generate_led_pwm(void);

#endif /* _H_ */
