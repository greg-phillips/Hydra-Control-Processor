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

/** @file leds.c
 *
 *  Created on: January 1, 2020
 *      Author: greg.phillips
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <memory.h>

#include "stm32g0xx_hal.h"

#include "system.h"
#include "structs.h"
#include "ck_time.h"
#include "led_color_defs.h"
#include "leds.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define BITS_OF_PWM             ( 8 )
#define MAX_LEVEL               ( 255 )
#define ENCODING_FACTOR         ( 4 )
#define NO_PWM_WORDS            ( LED_BAR_NO_LEDS * LEDS_NO_LEDS * 8 )
#define WIFI_SETUP_LED_TIME     ( 100 )
#define NO_TRANSISTION_STEPS    ( 10 )
#define WS2812B_RESET_TIME      ( 20 )		// Number of characters to generate 50uS Low
#define LEDS_NO_LEDS			( 3 )
#define DEFAULT_LOOP_COUNT		( 4 )
#define MIN_BREATH_LEVEL		( 50 )
#define NO_BREATH_STEPS			( 200 )
/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct {
    uint8_t bit_mask;
    uint16_t pin_mask;
    uint32_t pwm_output[ WS2812B_RESET_TIME +  NO_PWM_WORDS + WS2812B_RESET_TIME ];
    int16_t pwm_index;
    unsigned int leds_updated;
} led_display_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
void led_bar_test(void);
/******************************************************
 *               Variable Definitions
 ******************************************************/
bool volatile LED_display_update_completed;
static volatile led_display_t ld = { 0 };
uint16_t led_display[ LED_BAR_NO_LEDS ][ LEDS_NO_LEDS ];
extern hydra_status_t hs;
extern SPI_HandleTypeDef hspi2;
display_state_levels_t display_state_levels;
const unsigned char cie[256] = {
	0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 2, 2, 2, 2, 2, 2,
	2, 2, 2, 3, 3, 3, 3, 3, 3, 3,
	3, 4, 4, 4, 4, 4, 4, 5, 5, 5,
	5, 5, 6, 6, 6, 6, 6, 7, 7, 7,
	7, 8, 8, 8, 8, 9, 9, 9, 10, 10,
	10, 10, 11, 11, 11, 12, 12, 12, 13, 13,
	13, 14, 14, 15, 15, 15, 16, 16, 17, 17,
	17, 18, 18, 19, 19, 20, 20, 21, 21, 22,
	22, 23, 23, 24, 24, 25, 25, 26, 26, 27,
	28, 28, 29, 29, 30, 31, 31, 32, 32, 33,
	34, 34, 35, 36, 37, 37, 38, 39, 39, 40,
	41, 42, 43, 43, 44, 45, 46, 47, 47, 48,
	49, 50, 51, 52, 53, 54, 54, 55, 56, 57,
	58, 59, 60, 61, 62, 63, 64, 65, 66, 67,
	68, 70, 71, 72, 73, 74, 75, 76, 77, 79,
	80, 81, 82, 83, 85, 86, 87, 88, 90, 91,
	92, 94, 95, 96, 98, 99, 100, 102, 103, 105,
	106, 108, 109, 110, 112, 113, 115, 116, 118, 120,
	121, 123, 124, 126, 128, 129, 131, 132, 134, 136,
	138, 139, 141, 143, 145, 146, 148, 150, 152, 154,
	155, 157, 159, 161, 163, 165, 167, 169, 171, 173,
	175, 177, 179, 181, 183, 185, 187, 189, 191, 193,
	196, 198, 200, 202, 204, 207, 209, 211, 214, 216,
	218, 220, 223, 225, 228, 230, 232, 235, 237, 240,
	242, 245, 247, 250, 252, 255,
};
/******************************************************
 *               Function Definitions
 ******************************************************/
/**
  * @brief	Initialize the LED Display
  * @param  None
  * @retval : None
  */
void led_init(void)
{
	/*
	 * Set flag to indicate that the SPI is available to load new data
	 */
	LED_display_update_completed = true;
	hs.led_bar_state = LED_BAR_INIT;
    hs.ev_display_scan_mode = true;
    hs.led_bar_display_status = true;
    led_bar_scan_setup( green_blend_scan[ 0 ], green_blend_scan[ 1 ], green_blend_scan[ 2 ], 250 );
}
/**
  * @brief	Process the LED Display
  * @param  None
  * @retval : None
  */
void process_led_display( uint32_t current_time )
{
//	led_bar_test();
	led_bar_scan_mode_left( current_time );
}

/**
  * @brief set_led_bar_raw
  * @param  LED, Color Values R G B - 0 - 100 %
  * @retval : None
  */
void set_led_bar( uint8_t led, uint16_t red, uint16_t green, uint16_t blue )
{
	uint16_t red_level, green_level, blue_level;

	red_level 	= ( ( red * 255 ) ) / 100;
	green_level = ( ( green * 255 ) ) / 100;
	blue_level 	= ( ( blue * 255 ) ) / 100;

    set_led_bar_raw( led, red_level, green_level, blue_level );
}
/**
  * @brief set_led_bar_raw
  * @param  LED, Color Values R G B
  * @retval : None
  */
void set_led_bar_raw( uint8_t led, uint16_t raw_red_level, uint16_t raw_green_level, uint16_t raw_blue_level )
{
    /*
     * Set the raw value in the array and then recalculate the output pwm based on dim level
     *
     * Adjust based on https://www.ledsmagazine.com/smart-lighting-iot/color-tuning/article/16695054/understand-rgb-led-mixing-ratios-to-realize-optimal-color-in-signs-and-displays-magazine
     *
     * Color values and scale
     */
	if( raw_red_level > 255 )
		raw_red_level = 255;
	if( raw_green_level > 255 )
		raw_green_level = 255;
	if( raw_blue_level > 255 )
		raw_blue_level = 255;

    if( led < LED_BAR_NO_LEDS ) {
        led_display[ led ][ EV_LED_RED ] =  cie[ raw_red_level ];		// (uint8_t) (float) ( raw_red_level * ( 4.0 * ( 255 / 1020 ) ) );
        led_display[ led ][ EV_LED_GREEN ] =  cie[ raw_green_level ];	// (uint8_t) (float) ( raw_green_level * ( 3.0 * ( 255 / 1020 ) ) );
        led_display[ led ][ EV_LED_BLUE ] =  cie[ raw_blue_level ];	// (uint8_t) (float) ( raw_blue_level * ( 1.0 * ( 255 / 1020 ) ) );
    }
}
/**
  * @brief init_led_bar
  * @param  None
  * @retval : None
  */
void init_led_bar(void)
{
    led_bar_off();
}
/**
  * @brief led_bar_scan_setup
  * @param  None
  * @retval : None
  */
void led_bar_breath_setup( uint16_t red, uint16_t green, uint16_t blue, uint16_t transision_period )
{
    /*
     * Set Max levels
     */
	display_state_levels[ 0 ][ EV_LED_RED ] = red;
	display_state_levels[ 0 ][ EV_LED_GREEN ] = green;
	display_state_levels[ 0 ][ EV_LED_BLUE ] = blue;

    hs.tansistion_time = transision_period;
    hs.led_bar_state = LED_BAR_INIT;
    hs.ev_display_scan_mode = true;
    hs.led_bar_display_status = true;
    led_bar_off();
    hs.max_loop_count = 1;
}
/**
  * @brief led_bar_scan_setup
  * @param  None
  * @retval : None
  */
#define EVD_MAX     1.00
#define EVD_HIGH    0.70
#define EVD_MED     0.40
#define EVD_LOW     0.20
void led_bar_scan_setup( uint16_t red, uint16_t green, uint16_t blue, uint16_t transision_period )
{

    uint16_t i;
    /*
     * Set all to low first
     */
    for( i = 0; i < LED_BAR_NO_LEDS; i++ ) {
    	display_state_levels[ i ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_LOW );
    	display_state_levels[ i ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_LOW );
    	display_state_levels[ i ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_LOW );
    }
    /*
     * Calculate the entries for each BAR LED
     */
    /*
     *  0  ...  3   4     5    6    7     8   9       11
     * LOW ... LOW MED, HIGH, MAX, HIGH, MED LOW .... LOW
     */
    display_state_levels[ 4 ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_MED );
    display_state_levels[ 4 ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_MED );
    display_state_levels[ 4 ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_MED );

    display_state_levels[ 5 ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_HIGH );
    display_state_levels[ 5 ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_HIGH );
    display_state_levels[ 5 ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_HIGH );

    display_state_levels[ 6 ][ EV_LED_RED ] = red;
    display_state_levels[ 6 ][ EV_LED_GREEN ] = green;
    display_state_levels[ 6 ][ EV_LED_BLUE] = blue;

    display_state_levels[ 7 ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_HIGH );
    display_state_levels[ 7 ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_HIGH );
    display_state_levels[ 7 ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_HIGH );

    display_state_levels[ 8 ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_MED );
    display_state_levels[ 8 ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_MED );
    display_state_levels[ 8 ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_MED );

    hs.tansistion_time = transision_period;
    hs.led_bar_state = LED_BAR_INIT;
    hs.ev_display_scan_mode = true;
    hs.led_bar_display_status = true;
    led_bar_off();
    hs.max_loop_count = DEFAULT_LOOP_COUNT;
}
void led_bar_working_setup( uint16_t red, uint16_t green, uint16_t blue, uint16_t transision_period )
{

    uint16_t i;
    /*
     * Set all to low first
     */
    for( i = 0; i < LED_BAR_NO_LEDS; i++ ) {
    	display_state_levels[ i ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_LOW );
    	display_state_levels[ i ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_LOW );
    	display_state_levels[ i ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_LOW );
    }
    /*
     * Calculate the entries for each BAR LED
     */
    /*
     *  0   1   2   3   4   5   6   7   8   9   10  11
     * LOW MED LOW MED LOW MED LOW MED LOW MED LOW MED
     */
    display_state_levels[ 1 ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_MED );
    display_state_levels[ 1 ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_MED );
    display_state_levels[ 1 ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_MED );

    display_state_levels[ 3 ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_MED );
    display_state_levels[ 3 ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_MED );
    display_state_levels[ 3 ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_MED );

    display_state_levels[ 5 ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_MED );
    display_state_levels[ 5 ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_MED );
    display_state_levels[ 5 ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_MED );

    display_state_levels[ 7 ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_MED );
    display_state_levels[ 7 ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_MED );
    display_state_levels[ 7 ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_MED );

    display_state_levels[ 9 ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_MED );
    display_state_levels[ 9 ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_MED );
    display_state_levels[ 9 ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_MED );

    display_state_levels[ 11 ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_MED );
    display_state_levels[ 11 ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_MED );
    display_state_levels[ 11 ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_MED );

    hs.tansistion_time = transision_period;
    hs.led_bar_state = LED_BAR_INIT;
    hs.ev_display_scan_mode = true;
    hs.led_bar_display_status = true;
    led_bar_off();
    hs.max_loop_count = DEFAULT_LOOP_COUNT;
}
/**
  * @brief Show a display for Wi-Fi Setup
  * @param  None
  * @retval : None
  */
void led_bar_wifi_setup_mode(void)
{
    led_bar_scan_setup( green_blend_scan[ EV_LED_RED ], green_blend_scan[ EV_LED_GREEN ], green_blend_scan[ EV_LED_BLUE ], 50 );
}
/**
  * @brief Do the LED BAR Scanning function right
  * @param  None
  * @retval : None
  */
void led_bar_scan_mode_right( uint32_t current_time )
{
	int16_t i;
    uint16_t index;

    /*
     * Show a Wi Fi Setup display
     */
    switch( hs.led_bar_state ) {
        case LED_BAR_INIT :
            hs.led_bar_display_state = 0;
            hs.led_bar_last_update = current_time;
            led_bar_off();      // Start in known state
            hs.led_bar_state = LED_BAR_SCAN_RIGHT;
            break;
        case LED_BAR_SCAN_RIGHT :
            if( imx_is_later( current_time, hs.led_bar_last_update + hs.tansistion_time ) ) {
                for( i = 0; i < LED_BAR_NO_LEDS; i++ ) {
                	index = hs.led_bar_display_state + i;
                	if( index >= LED_BAR_NO_LEDS )
                		index = index - LED_BAR_NO_LEDS;	// Wrap around
                	set_led_bar_raw( i, display_state_levels[ index ][ EV_LED_RED ],
                			display_state_levels[ index ][ EV_LED_GREEN ],
							display_state_levels[ index ][ EV_LED_BLUE ] );
                }
                update_led_bar();
                hs.led_bar_last_update = current_time;
                /*
                 * Check if we have done a full lap
                 */
                if( hs.led_bar_display_state == 0 ) {
                	hs.led_bar_display_state = LED_BAR_NO_LEDS - 1;
                } else {
                    hs.led_bar_display_state -= 1;
                }
            }
            break;
        default :
        	hs.led_bar_state = LED_BAR_INIT;
    }
}
/**
  * @brief Do the LED BAR Scanning function left
  * @param  None
  * @retval : None
  */
void led_bar_scan_mode_left( uint32_t current_time )
{
	int16_t i;
    uint16_t index;

    /*
     * Show a Wi Fi Setup display
     */
    switch( hs.led_bar_state ) {
        case LED_BAR_INIT :
            hs.led_bar_display_state = 0;
            hs.led_bar_last_update = current_time;
            led_bar_off();      // Start in known state
            hs.loop_count = 0;
            hs.led_bar_state = LED_BAR_SCAN_LEFT;
            break;
        case LED_BAR_SCAN_LEFT :
            if( imx_is_later( current_time, hs.led_bar_last_update + hs.tansistion_time ) ) {
                for( i = 0; i < LED_BAR_NO_LEDS; i++ ) {
                	index = hs.led_bar_display_state + i;
                	if( index >= LED_BAR_NO_LEDS )
                		index = index - LED_BAR_NO_LEDS;	// Wrap around
                	set_led_bar_raw( i, display_state_levels[ index ][ EV_LED_RED ],
                			display_state_levels[ index ][ EV_LED_GREEN ],
							display_state_levels[ index ][ EV_LED_BLUE ]);
                }
                update_led_bar();
                hs.led_bar_last_update = current_time;
                hs.led_bar_display_state += 1;
                /*
                 * Check if we have done a full lap
                 */
                if( hs.led_bar_display_state >= LED_BAR_NO_LEDS ) {
                	hs.led_bar_display_state = 0;	// Do it again
                }
            }
            break;
        default :
        	hs.led_bar_state = LED_BAR_INIT;
            break;
    }
}
/**
  * @brief Do the LED BAR Scanning alternate forward / backwards function 2 X
  * @param  None
  * @retval : None
  */
void led_bar_scan_mode_dual( uint32_t current_time )
{
	int16_t i;
    uint16_t index;

    /*
     * Show a Wi Fi Setup display
     */
    switch( hs.led_bar_state ) {
        case LED_BAR_INIT :
            hs.led_bar_display_state = 0;
            hs.led_bar_last_update = current_time;
            led_bar_off();      // Start in known state
            hs.loop_count = 0;
            hs.led_bar_state = LED_BAR_SCAN_LEFT;
            break;
        case LED_BAR_SCAN_LEFT :
            if( imx_is_later( current_time, hs.led_bar_last_update + hs.tansistion_time ) ) {
                for( i = 0; i < LED_BAR_NO_LEDS; i++ ) {
                	index = hs.led_bar_display_state + i;
                	if( index >= LED_BAR_NO_LEDS )
                		index = index - LED_BAR_NO_LEDS;	// Wrap around
                	set_led_bar_raw( i, display_state_levels[ index ][ EV_LED_RED ],
                			display_state_levels[ index ][ EV_LED_GREEN ],
							display_state_levels[ index ][ EV_LED_BLUE ]);
                }
                update_led_bar();
                hs.led_bar_last_update = current_time;
                hs.led_bar_display_state += 1;
                /*
                 * Check if we have done a full lap
                 */
                if( hs.led_bar_display_state >= LED_BAR_NO_LEDS ) {
                	hs.loop_count += 1;
                	if( hs.loop_count >= hs.max_loop_count ) {
                        hs.led_bar_display_state = LED_BAR_NO_LEDS - 1;
                        hs.loop_count = 0;
                        hs.led_bar_state = LED_BAR_SCAN_RIGHT;
                	} else {
                		hs.led_bar_display_state = 0;	// Do it again
                	}
                }
            }
            break;
        case LED_BAR_SCAN_RIGHT :
            if( imx_is_later( current_time, hs.led_bar_last_update + hs.tansistion_time ) ) {
                for( i = 0; i < LED_BAR_NO_LEDS; i++ ) {
                	index = hs.led_bar_display_state + i;
                	if( index >= LED_BAR_NO_LEDS )
                		index = index - LED_BAR_NO_LEDS;	// Wrap around
                	set_led_bar_raw( i, display_state_levels[ index ][ EV_LED_RED ],
                			display_state_levels[ index ][ EV_LED_GREEN ],
							display_state_levels[ index ][ EV_LED_BLUE ] );
                }
                update_led_bar();
                hs.led_bar_last_update = current_time;
                /*
                 * Check if we have done a full lap
                 */
                if( hs.led_bar_display_state == 0 ) {
                	hs.loop_count += 1;
                	if( hs.loop_count >= hs.max_loop_count ) {
                		hs.led_bar_state = LED_BAR_SCAN_LEFT;
                		hs.loop_count = 0;
                	} else {
                		hs.led_bar_display_state = LED_BAR_NO_LEDS - 1;	// Do it again
                	}
                } else {
                    hs.led_bar_display_state -= 1;
                }
            }
            break;
        default :
        	hs.led_bar_state = LED_BAR_INIT;
            break;
    }
}
/**
  * @brief Do the LED BAR Scanning alternate forward / backwards function 2 X
  * @param  None
  * @retval : None
  */
void led_bar_breath_mode( uint32_t current_time )
{
	int16_t i;
	float factor;
    /*
     * Show a Wi Fi Setup display
     */
    switch( hs.led_bar_state ) {
        case LED_BAR_INIT :
            hs.led_bar_display_state = MIN_BREATH_LEVEL;
            hs.led_bar_last_update = current_time;
            led_bar_off();      // Start in known state
            hs.led_bar_state = LED_BAR_SCAN_LEFT;
            break;
        case LED_BAR_SCAN_LEFT :
            if( imx_is_later( current_time, hs.led_bar_last_update + hs.tansistion_time ) ) {
                for( i = 0; i < LED_BAR_NO_LEDS; i++ ) {
                	factor = (float)( hs.led_bar_display_state ) / (float) (NO_BREATH_STEPS);
                	set_led_bar_raw( i, (uint16_t) ( (float)display_state_levels[ 0 ][ EV_LED_RED ] * factor ),
                			(uint16_t) ( (float) display_state_levels[ 0 ][ EV_LED_GREEN ] * factor ),
							(uint16_t) ( (float) display_state_levels[ 0 ][ EV_LED_BLUE ] * factor ) );
                }
                update_led_bar();
                hs.led_bar_last_update = current_time;
                hs.led_bar_display_state += 1;
                /*
                 * Check if we have done a full lap
                 */
                if( hs.led_bar_display_state >= NO_BREATH_STEPS ) {
                	hs.led_bar_display_state = NO_BREATH_STEPS - 1;
                	hs.led_bar_state = LED_BAR_SCAN_RIGHT;
                }
            }
            break;
        case LED_BAR_SCAN_RIGHT :
            if( imx_is_later( current_time, hs.led_bar_last_update + hs.tansistion_time ) ) {
                for( i = 0; i < LED_BAR_NO_LEDS; i++ ) {
                	factor = (float)( hs.led_bar_display_state ) / (float) (NO_BREATH_STEPS);
                	set_led_bar_raw( i, (uint16_t) ( (float)display_state_levels[ 0 ][ EV_LED_RED ] * factor ),
                			(uint16_t) ( (float) display_state_levels[ 0 ][ EV_LED_GREEN ] * factor ),
							(uint16_t) ( (float) display_state_levels[ 0 ][ EV_LED_BLUE ] * factor ) );
                }
                update_led_bar();
                hs.led_bar_last_update = current_time;
                /*
                 * Check if we have done a full lap
                 */
                if( hs.led_bar_display_state == MIN_BREATH_LEVEL ) {
                	hs.led_bar_state = LED_BAR_SCAN_LEFT;
                } else {
                    hs.led_bar_display_state -= 1;
                }
            }
            break;
        default :
        	hs.led_bar_state = LED_BAR_INIT;
            break;
    }
}
/**
  * @brief led_bar_off
  * @param  None
  * @retval : None
  */
void led_bar_off(void)
{
    memset( &led_display, 0x00, sizeof( led_display ) );
    update_led_bar();
}/*
#define NZR_0   0x1     // 1 0 0 - reversed = 0 0 1
#define NZR_1   0x3     // 1 1 0 - reversed = 0 1 1

#define NZR_0   0x8     // 1 0 0 0
#define NZR_1   0xE     // 1 1 1 0
*/
#define NZR_0   0b11111111110000000000000000000000
#define NZR_1   0b11111111111111111111000000000000
/**
  * @brief Generate the PWM levels based on lux level
  * @param  None
  * @retval : None
  */
void generate_led_pwm(void)
{
    uint8_t led_level, mask;
    int16_t i, j, index;
    float lux_scale;

    if( hs.valid_lux_level == true ) {
        lux_scale = 0.5;        // determine correct levels later - use a calculation
    } else
        lux_scale = 0.5;

    /*
     * Output PWM Bits are sent for the last LED first and each of the 24 bits are sent in order of G R B bit order 7-0
     *
     * The Array contains additional byte to generate reload of data
     */
    memset( (void *) &ld.pwm_output, 0x00, sizeof( ld.pwm_output ) );
    index = WS2812B_RESET_TIME;   // Start after some preamble
    for( i = 0; i < LED_BAR_NO_LEDS;  i++ ) {
        for( j = 0; j < LEDS_NO_LEDS; j++ ) {
            led_level = (uint8_t) ( lux_scale * (float) led_display[ i ][ j ] );
            for( mask = 0x80; mask != 0 ; mask >>= 1 ) {
                if( ( led_level & mask ) == 0x00 ) {
                    ld.pwm_output[ index++ ] = NZR_0;
                } else {
                    ld.pwm_output[ index++ ] = NZR_1;
                }
            }
        }
    }
}
/**
  * @brief update_led_bar
  * @param  None
  * @retval : None
  */
void update_led_bar(void)
{
	/*
	 * Generate PWM Waveform
	 */
	generate_led_pwm();
	/*
	 * Take the data that has been prepared using generate pwm and output over SPI 2
	 */
	LED_display_update_completed = false;
	/*
	 * Initiate Transfer
	 */
	HAL_SPI_Transmit_DMA( &hspi2, (uint8_t *) ld.pwm_output, sizeof( ld.pwm_output ) );
	/*
	 * Wait for it to complete - Later make this checked in main loop for LED management to speed operations up
	 */
	while( LED_display_update_completed == false )
		;
}
/**
  * @brief led bar test
  * @param  None
  * @retval : None
  */
void led_bar_test(void)
{
    uint16_t j, i, color;

    color = 0;

    led_bar_off();
    HAL_Delay( 2000 );
//    for( color = 0; color < 3; i++ ) {
        led_bar_off();
        for( j = 1; j < 256; j ++ ) {
            set_led_bar_raw( 0, j, 0, 0 );
            set_led_bar_raw( 6, j, 0, 0 );
            set_led_bar_raw( 11, j, 0, 0 );
            update_led_bar();
            HAL_Delay( 10 );
        }
//    }
	return;
/*
 *             for( i = 0; i < LED_BAR_NO_LEDS; i++ ) {
                if( color == EV_LED_RED ) {
                    set_led_bar_raw( i, j, 0, 0 );
                } else if( color == EV_LED_GREEN ) {
                	set_led_bar_raw( i, 0, j, 0 );
                } else {
                	set_led_bar_raw( i, 0, 0, j );
                }
            }
 *
 */
    for( j = 0; j < 3; j ++ ) {
        for( i = 0; i < LED_BAR_NO_LEDS; i++ ) {
            if( color == EV_LED_RED ) {
                set_led_bar( i, 100, 0, 0 );
//                printf( "Red\r\n" );
                update_led_bar();
                HAL_Delay( 1000 );
            } else if( color == EV_LED_GREEN ) {
                set_led_bar( i, 0, 100, 0 );
//                printf( "Green\r\n" );
            } else {
                set_led_bar( i, 0, 0, 100 );
//                printf( "Blue\r\n" );
            }
        }
        update_led_bar();
        HAL_Delay( 2000 );
        color += 1;
        if( color >= LEDS_NO_LEDS )
            color = 0;
        led_bar_off();
    }
}

