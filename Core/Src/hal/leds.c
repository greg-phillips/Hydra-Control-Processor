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
#define NO_PWM_WORDS            ( LED_BAR_NO_LEDS * LEDS_NO_LEDS )
#define WIFI_SETUP_LED_TIME     ( 100 )
#define NO_TRANSISTION_STEPS    ( 10 )
#define WS2812B_RESET_TIME      ( 60 )		// Number of characters to generate 50uS Low
#define LEDS_NO_LEDS			( 3 )

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
    led_bar_setup( blue_scan[ 0 ], blue_scan[ 1 ], blue_scan[ 2 ], 1000 );
}
/**
  * @brief	Process the LED Display
  * @param  None
  * @retval : None
  */
void process_led_display( uint32_t current_time )
{
	led_bar_test();
//	led_bar_scan_mode( current_time );
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
    if( led < LED_BAR_NO_LEDS ) {
        led_display[ led ][ EV_LED_RED ] =  raw_red_level;		// (uint8_t) (float) ( raw_red_level * ( 4.0 * ( 255 / 1020 ) ) );
        led_display[ led ][ EV_LED_GREEN ] =  raw_green_level;	// (uint8_t) (float) ( raw_green_level * ( 3.0 * ( 255 / 1020 ) ) );
        led_display[ led ][ EV_LED_BLUE ] =  raw_blue_level;	// (uint8_t) (float) ( raw_blue_level * ( 1.0 * ( 255 / 1020 ) ) );
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
  * @brief led_bar_setup
  * @param  None
  * @retval : None
  */
display_state_levels_t display_state_levels;
#define EVD_MAX     1.00
#define EVD_HIGH    0.70
#define EVD_MED     0.40
#define EVD_LOW     0.10
void led_bar_setup( uint16_t red, uint16_t green, uint16_t blue, uint16_t transision_period )
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
}
/**
  * @brief Show a display for Wi-Fi Setup
  * @param  None
  * @retval : None
  */
void led_bar_wifi_setup_mode(void)
{
    led_bar_setup( green_blend_scan[ EV_LED_RED ], green_blend_scan[ EV_LED_GREEN ], green_blend_scan[ EV_LED_BLUE ], 50 );
}
/**
  * @brief Do the LED BAR Scanning function
  * @param  None
  * @retval : None
  */
void led_bar_scan_mode( uint32_t current_time )
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
                    hs.led_bar_display_state = 0;
                    hs.led_bar_state = LED_BAR_SCAN_RIGHT;
                }
            }
            break;
        case LED_BAR_SCAN_RIGHT :
            if( imx_is_later( current_time, hs.led_bar_last_update + hs.tansistion_time ) ) {
                for( i = ( LED_BAR_NO_LEDS - 1 ); i > 0; i-- ) {
                	index = hs.led_bar_display_state + i;
                	if( index >= LED_BAR_NO_LEDS )
                		index = index - LED_BAR_NO_LEDS;	// Wrap around
                	set_led_bar_raw( i, display_state_levels[ index ][ EV_LED_RED ],
                			display_state_levels[ index ][ EV_LED_GREEN ],
							display_state_levels[ index ][ EV_LED_BLUE ] );
                }
                update_led_bar();
                hs.led_bar_last_update = current_time;
                hs.led_bar_display_state += 1;
                /*
                 * Check if we have done a full lap
                 */
                if( hs.led_bar_display_state >= LED_BAR_NO_LEDS ) {
                    hs.led_bar_display_state = 0;
                    hs.led_bar_state = LED_BAR_SCAN_LEFT;
                }
            }
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
*/
#define NZR_0   0x8     // 1 0 0 0
#define NZR_1   0xE     // 1 1 1 0

/**
  * @brief Generate the PWM levels based on lux level
  * @param  None
  * @retval : None
  */
void generate_led_pwm(void)
{
    uint8_t led_level, mask;
    int16_t i, j, index;
    uint32_t output_value;
    float lux_scale;

    if( hs.valid_lux_level == true ) {
        lux_scale = 1.0;        // determine correct levels later - use a calculation
    } else
        lux_scale = 1.0;

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
//            printf( "LED Display[ %u ][ %u ] = 0x%02x\r\n", i, j, (uint16_t) pwm_output[ i ] );
            output_value = 0;
            for( mask = 0x80; mask != 0 ; mask >>= 1 ) {
            	output_value <<= 4;
                if( ( led_level & mask ) == 0x00 ) {
                	output_value |= (uint32_t) ( NZR_0 );
                } else {
                	output_value |= (uint32_t) ( NZR_1 );
                }
            }
//            printf( "Index: %u\r\n", index );
            ld.pwm_output[ index++ ] = output_value;
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

