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
#ifndef STRUCTS_H_
#define STRUCTS_H_

#include <stdbool.h>
#include "system.h"
#include "kalman_filter.h"

/** @file structs.h
 *
 *  Created on: December 30, 2019
 *      Author: greg.phillips
 *
 */
/*
 *	Defines for structures used in the Hydra Control processor
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
typedef enum {
    EV_SW_INIT,
    EV_SW_MONITOR_SWITCH,
    EV_SW_DETECTED,
    EV_SW_WAKEUP,
    EV_SW_PREPARE_PROVISIONING,
    EV_SW_START_PROVISION,
    EV_SW_WATI_FOR_PROVISION,
    EV_SW_COMPLETE_PROVISION,
    EV_SW_PREPARE_SHUTDOWN,
    EV_SW_SHUTDOWN_IN_PROGRESS,
    EV_SW_SHUTDOWN,
    EV_SW_NO_STATES
} hydra_sw_states_t;

typedef enum {
    DEBOUNCE_INIT,
    DEBOUNCE_WAIT_FOR_SWITCH,
    DEBOUNCE_WAIT_FOR_DEBOUNCE,
    DEBOUNCE_WAIT_FOR_OFF,
} hydra_sw_debounce_t;

typedef enum {
    HD_INIT,
    HD_SHOW_SENSORS,
    HD_SHOW_HEALTH,
}  hydra_display_t;

typedef uint16_t display_state_levels_t[ NO_DISPLAY_STATES ][ LED_BAR_NO_LEDS ][ LEDS_NO_LEDS ];

typedef enum {
    LED_BAR_INIT,
    LED_BAR_SCAN_LEFT,
    LED_BAR_TRANISTION_LEFT,
    LED_BAR_SCAN_RIGHT,
    LED_BAR_TRANISTION_RIGHT,
} led_bar_states_t;

typedef struct hydra_status {
	/*
	 * LED Display variables
	 */
	uint16_t led_bar_display_state;
	uint32_t transistion_time;
	led_bar_states_t led_bar_state;
    uint32_t led_bar_last_update;
    /*
     * User Input State machine
     */
    hydra_sw_states_t ui_state;
    hydra_sw_debounce_t debounce_state;
    uint32_t input_debounce_time;
    uint32_t last_switch_time;
    uint16_t power_down_led;
    uint16_t transisition_step;
    uint16_t tansistion_time;
    /*
     * Hydra Display
     */
    hydra_display_t hydra_display_state;
    /*
     * General Timers
     */
    // wiced_utc_time_t reboot_time;
    /*
     * Current State of Sensors
     */
    /*
     * Basic sensors
     */
    uint32_t current_co2_level;
    uint32_t current_tvoc_level;
    uint32_t current_humidity_level;
    float current_temp_level;
    uint32_t current_lux_level;
    uint32_t current_ir_lux_level;
    /*
     * Environmental Sensor BME680
     */
    float current_iaq;
    uint16_t current_iaq_accuracy;
    float current_temperature;
    float current_humidity;
    float current_pressure;
    float current_altitude;
    float current_raw_temperature;
    float current_raw_humidity;
    float current_gas;
    float current_co2_equivalent;
    float current_breath_voc_equivalent;
    /*
     * Particluate Matter Sensor
     */
    float current_PM_1_0;
    float current_PM_2_5;
    float current_PM_4_0;
    float current_PM_10_0;
    float current_no_PM_0_5;
    float current_no_PM_1_0;
    float current_no_PM_2_5;
    float current_no_PM_4_0;
    float current_no_PM_10_0;
    float current_typical_particle;
    uint32_t current_PM_index;
    /*
     * Kalaman Filters for PM Sensor
     */
    kalman_filter_t kf_PM_1_0;
    kalman_filter_t kf_PM_2_5;
    kalman_filter_t kf_PM_4_0;
    kalman_filter_t kf_PM_10_0;
    kalman_filter_t kf_no_PM_0_5;
    kalman_filter_t kf_no_PM_1_0;
    kalman_filter_t kf_no_PM_2_5;
    kalman_filter_t kf_no_PM_4_0;
    kalman_filter_t kf_no_PM_10_0;
    kalman_filter_t kf_typical_particle;
    /*
     * Counts
     */
    uint32_t false_temp_count;
    uint32_t false_humidity_count;
    uint32_t lux_noise_count;
    uint32_t last_lux_level;
    uint32_t false_lux_count;
    uint32_t iaq_baseline;
    uint32_t spi_errors;

    /*
     * Set if data is valid
     */
    unsigned int led_bar_display_status         : 1;    // Should we show status on the level bar - Off when operating on Battery
    unsigned int ev_display_scan_mode           : 1;    // Running led "Scanning process"
    unsigned int ignore_save                    : 1;
    unsigned int valid_co2_level                : 1;
    unsigned int valid_tvoc_level               : 1;
    unsigned int valid_humidity_level           : 1;
    unsigned int valid_temp_level               : 1;
    unsigned int valid_lux_level                : 1;
    unsigned int valid_ir_lux_level             : 1;
    unsigned int valid_particulate_matter       : 1;
    unsigned int valid_environmental_sensor     : 1;
    unsigned int reboot                         : 1;    // Reboot next time thru do everything
    unsigned int soft_reboot                    : 1;    // Did we just do a soft reboot - qla_status restored from RAM

} hydra_status_t;
/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

#endif /* STRUCTS_H_ */
