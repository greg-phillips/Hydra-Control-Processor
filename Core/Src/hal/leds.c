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
#define ENCODING_FACTOR         ( 3 )
#define NO_PWM_BYTES            ( ( LED_BAR_NO_LEDS * LEDS_NO_LEDS * BITS_OF_PWM * ENCODING_FACTOR ) / 8 )
#define WIFI_SETUP_LED_TIME     ( 100 )
#define NO_TRANSISTION_STEPS    ( 10 )
#define WS2812B_RESET_TIME      ( 50 )
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
    uint8_t pwm_output[ NO_PWM_BYTES ];
    int16_t pwm_index;
    unsigned int leds_updated;
} led_display_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
bool LED_display_update_completed;
static volatile led_display_t ld;
uint16_t led_display[ LED_BAR_NO_LEDS ][ LEDS_NO_LEDS ];
extern hydra_status_t hs;
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
}
/**
  * @brief update_led_bar
  * @param  None
  * @retval : None
  */
void update_led_bar(void)
{

}
/**
  * @brief	Process the LED Display
  * @param  None
  * @retval : None
  */
void process_led_display(void)
{

}

/**
  * @brief set_led_bar_raw
  * @param  LED, Color Values R G B - 0 - 100 %
  * @retval : None
  */
void set_led_bar( uint8_t led, float red_level, float green_level, float blue_level )
{
    set_led_bar_raw( led, (uint16_t) ( ( red_level / 100 ) * 255.0  ), (uint16_t) ( ( green_level / 100 ) * 255.0 ), (uint16_t) ( ( blue_level / 100 ) *  255.0 ) );
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
    if( led < ( LED_BAR_NO_LEDS - 1) ) {
        led_display[ led ][ EV_LED_RED ] =  (uint8_t) (float) ( raw_red_level * ( 4.0 * ( 255 / 1020 ) ) );
        led_display[ led ][ EV_LED_GREEN ] =  (uint8_t) (float) ( raw_green_level * ( 3.0 * ( 255 / 1020 ) ) );
        led_display[ led ][ EV_LED_BLUE ] =  (uint8_t) (float) ( raw_blue_level * ( 1.0 * ( 255 / 1020 ) ) );
        generate_led_pwm();
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

    uint16_t i, j;
    /*
     * Set all to low first
     */
    for( i = 0; i < NO_DISPLAY_STATES; i++ ) {
        for( j = 0; j < LED_BAR_NO_LEDS; j++ ) {
            display_state_levels[ i ][ j ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_LOW );
            display_state_levels[ i ][ j ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_LOW );
            display_state_levels[ i ][ j ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_LOW );
        }
    }
    /*
     * Calculate the entries for each BAR LED
     */
    for( i = 0; i < NO_DISPLAY_STATES; i++ ) {
        /*
         * LOW ... MED, HIGH, MAX, HIGH, MED .... LOW
         */
        if( i > 1 ) {
            display_state_levels[ i ][ i - 2 ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_MED );
            display_state_levels[ i ][ i - 2 ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_MED );
            display_state_levels[ i ][ i - 2 ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_MED );
        }
        if( i > 0 ) {
            display_state_levels[ i ][ i - 1 ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_HIGH );
            display_state_levels[ i ][ i - i ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_HIGH );
            display_state_levels[ i ][ i - 1 ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_HIGH );
        }
        display_state_levels[ i ][ i ][ EV_LED_RED ] = red;
        display_state_levels[ i ][ i ][ EV_LED_GREEN ] = green;
        display_state_levels[ i ][ i ][ EV_LED_BLUE] = blue;

        if( i <= ( LED_BAR_NO_LEDS - 2 ) ) {
            display_state_levels[ i ][ i + 1 ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_HIGH );
            display_state_levels[ i ][ i + 1 ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_HIGH );
            display_state_levels[ i ][ i + 1 ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_HIGH );
        }
        if( i <= ( LED_BAR_NO_LEDS - 3 ) ) {
            display_state_levels[ i ][ i + 2 ][ EV_LED_RED ] = (uint16_t) ( (float) red * EVD_MED );
            display_state_levels[ i ][ i + 2 ][ EV_LED_GREEN ] = (uint16_t) ( (float) green * EVD_MED );
            display_state_levels[ i ][ i + 2 ][ EV_LED_BLUE ] = (uint16_t) ( (float) blue * EVD_MED );
        }
    }
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
    uint16_t i;
    float red_transisition;
    float green_transisition;
    float blue_transisition;

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
                        set_led_bar( i, display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_RED ],
                                        display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_GREEN ],
                                        display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_BLUE ]);
                }
                update_led_bar();
                hs.led_bar_last_update = current_time;
                if( hs.led_bar_display_state >= ( NO_DISPLAY_STATES - 1 ) ) {
                    hs.led_bar_display_state = NO_DISPLAY_STATES - 1;
                    hs.led_bar_state = LED_BAR_SCAN_RIGHT;
                } else {
                    hs.led_bar_state = LED_BAR_TRANISTION_LEFT;
                    hs.transisition_step = 1;
                }
            }
            break;
        case LED_BAR_TRANISTION_LEFT :
            /*
             * Take NO_TRANSISTION_STEPS to transition to next state
             */
            if( imx_is_later( current_time, hs.led_bar_last_update + hs.tansistion_time ) ) {

                for( i = 0; i < LED_BAR_NO_LEDS; i++ ) {
                    red_transisition = ( (float) ( display_state_levels[ hs.led_bar_display_state + 1 ][ i ][ EV_LED_RED ] -
                                                   display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_RED ] )
                                                     / NO_TRANSISTION_STEPS ) * hs.transisition_step;
                    green_transisition = ( (float) ( display_state_levels[ hs.led_bar_display_state + 1 ][ i ][ EV_LED_GREEN ] -
                                                     display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_GREEN ] )
                                                     / NO_TRANSISTION_STEPS ) * hs.transisition_step;
                    blue_transisition = ( (float) ( display_state_levels[ hs.led_bar_display_state + 1 ][ i ][ EV_LED_BLUE ] -
                                                    display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_BLUE ] )
                                                     / NO_TRANSISTION_STEPS ) * hs.transisition_step;
                    set_led_bar( i, display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_RED ] + red_transisition,
                            display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_GREEN ] + green_transisition,
                            display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_BLUE ] + blue_transisition );
                }
                update_led_bar();
                hs.led_bar_last_update = current_time;
                hs.transisition_step +=  1;
                if( hs.transisition_step >= NO_TRANSISTION_STEPS ) {
                    hs.led_bar_display_state += 1;
                    hs.led_bar_state = LED_BAR_SCAN_LEFT;
                }
            }
            break;
        case LED_BAR_SCAN_RIGHT :
            if( imx_is_later( current_time, hs.led_bar_last_update + hs.tansistion_time ) ) {
                for( i = 0; i < LED_BAR_NO_LEDS; i++ ) {
                        set_led_bar( i, display_state_levels[ hs.led_bar_display_state ][ i ][ 0 ],
                                        display_state_levels[ hs.led_bar_display_state ][ i ][ 1 ],
                                        display_state_levels[ hs.led_bar_display_state ][ i ][ 2 ] );
                }
                update_led_bar();
                hs.led_bar_last_update = current_time;
                if( hs.led_bar_display_state == 0 ) {
                    hs.led_bar_display_state = 0;
                    hs.led_bar_state = LED_BAR_SCAN_LEFT;
                } else {
                    hs.led_bar_state = LED_BAR_TRANISTION_RIGHT;
                    hs.transisition_step = 1;
                }
            }
            break;
        case LED_BAR_TRANISTION_RIGHT :
            /*
             * Take NO_TRANSISTION_STEPS to transition to next state
             */
            if( imx_is_later( current_time, hs.led_bar_last_update + hs.tansistion_time ) ) {
                for( i = 0; i < LED_BAR_NO_LEDS; i++ ) {
                    red_transisition = ( (float) ( display_state_levels[ hs.led_bar_display_state - 1 ][ i ][ EV_LED_RED ] -
                                                   display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_RED ] )
                                                     / NO_TRANSISTION_STEPS ) * hs.transisition_step;
                    green_transisition = ( (float) ( display_state_levels[ hs.led_bar_display_state - 1 ][ i ][ EV_LED_GREEN ] -
                                                     display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_GREEN ] )
                                                     / NO_TRANSISTION_STEPS ) * hs.transisition_step;
                    blue_transisition = ( (float) ( display_state_levels[ hs.led_bar_display_state - 1 ][ i ][ EV_LED_BLUE ] -
                                                    display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_BLUE ] )
                                                     / NO_TRANSISTION_STEPS ) * hs.transisition_step;
                    set_led_bar( i, display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_RED ] + red_transisition,
                            display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_GREEN ] + green_transisition,
                            display_state_levels[ hs.led_bar_display_state ][ i ][ EV_LED_BLUE ] + blue_transisition );
                }
                update_led_bar();
                hs.led_bar_last_update = current_time;
                hs.transisition_step +=  1;
                if( hs.transisition_step >= NO_TRANSISTION_STEPS ) {
                    hs.led_bar_display_state -= 1;
                    hs.led_bar_state = LED_BAR_SCAN_RIGHT;
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
    generate_led_pwm();
    update_led_bar();
}

#define NZR_0   0x1     // 1 0 0 - reversed = 0 0 1
#define NZR_1   0x3     // 1 1 0 - reversed = 0 1 1
/**
  * @brief Generate the PWM levels based on lux level
  * @param  None
  * @retval : None
  */
void generate_led_pwm(void)
{
    uint8_t led_level;
    int16_t i, j, k, index;
    uint32_t output_value, mask;
    float lux_scale;

    if( hs.valid_lux_level == true ) {
        lux_scale = 1.0;        // determine correct levels later - use a calculation
    } else
        lux_scale = 1.0;

    /*
     * Output PWM Bits are sent for the last LED first and each of the 24 bits are sent in order of G R B bit order 7-0
     */
    memset( (void *) &ld.pwm_output, 0x00, sizeof( ld.pwm_output ) );
    index = NO_PWM_BYTES - 1;   // Start at end and back fill
    for( i = ( LED_BAR_NO_LEDS - 1);  i >= 0; i-- ) {
        for( j = ( LEDS_NO_LEDS - 1); j >= 0; j-- ) {
            led_level = (uint8_t) ( lux_scale * (float) led_display[ i ][ j ] );
//            printf( "LED Display[ %u ][ %u ] = 0x%02x\r\n", i, j, (uint16_t) pwm_output[ i ] );
            output_value = 0;
            for( k = 0; k < BITS_OF_PWM; k++ ) {
                if( ( led_level & ( 1 << k ) ) == 0x00 ) {
                    mask= (uint32_t) ( NZR_0 << ( k * 3 ) );
//                    printf( "[%u] Mask low: 0x%08lx\r\n", k, mask );
                    output_value |= mask;
                } else {
                    mask = (uint32_t) ( NZR_1 << ( k * 3 ) );
//                    printf( "[%u] Mask high: 0x%08lx\r\n", k, mask );
                    output_value |= mask;
                }
            }
//            printf( "Index: %u\r\n", index );
            ld.pwm_output[ index-- ] = output_value & 0x000000FF;
            ld.pwm_output[ index-- ] = ( output_value & 0x0000FF00 ) >> 8;
            ld.pwm_output[ index-- ] = ( output_value & 0x00FF0000 ) >> 16;
        }
    }
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

    for( j = 0; j < 3; j ++ ) {
        for( i = 0; i < LED_BAR_NO_LEDS; i++ ) {
            if( color == EV_LED_RED ) {
                set_led_bar( i, 100, 0, 0 );
//                printf( "Red\r\n" );
            } else if( color == EV_LED_GREEN ) {
                set_led_bar( i, 0, 100, 0 );
//                printf( "Green\r\n" );
            } else {
                set_led_bar( i, 0, 0, 100 );
//                printf( "Blue\r\n" );
            }
        }
        if( color >= LED_BAR_NO_LEDS )
            color = 0;
        else
            color += 1;
        update_led_bar();
//        wiced_rtos_delay_milliseconds( 2000 );
    }
}

