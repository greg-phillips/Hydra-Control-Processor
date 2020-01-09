/*
 * Copyright 2019, iMatrix Systems, Inc.. All Rights Reserved.
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

/** @file sgp30_interface.c
 *
 *  Created on: October 29, 2019
 *      Author: greg.phillips
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "structs.h"
#include "sgp30.h"
#include "sensirion_common.h"

/******************************************************
 *                      Macros
 ******************************************************/
#ifdef PRINT_DEBUGS_FOR_SENSORS
    #undef PRINTF
    #define PRINTF(...) if( ( evc.log_messages & DEBUGS_FOR_SENSORS ) != 0x00 ) imx_cli_print(__VA_ARGS__)
#elif !defined PRINTF
    #define PRINTF(...)
#endif

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
extern hydra_status_t hs;
const char *SGP_DRV_VERSION_STR = "Version 1.0 10-28-2019";
/******************************************************
 *               Function Definitions
 ******************************************************/
/**
  * @brief Initialize the SPS30 Sensor
  * @param  None
  * @retval : None
  */
bool init_SGP30(void)
{
    uint16_t err, probe;
    uint16_t feature_set_version;
    uint8_t product_type;
    uint16_t ethanol_raw_signal;
    uint16_t h2_raw_signal;

    const char *driver_version = sgp30_get_driver_version();
    if (driver_version) {
        printf("SGP30 driver version %s\r\n", driver_version);
    } else {
        printf("fatal: Getting driver version failed\r\n");
        return false;
    }

    if( ( probe = sgp30_probe() ) != STATUS_OK) {
        if (probe == SGP30_ERR_UNSUPPORTED_FEATURE_SET)
            printf( "Your sensor needs at least feature set version 1.0 (0x20)\r\n" );
        printf("SGP sensor probing failed\r\n");
        return false;
    }
    printf("SGP sensor probing successful\r\n");

    err = sgp30_get_feature_set_version(&feature_set_version, &product_type);
    if (err == STATUS_OK) {
        printf("Feature set version: %u\r\n", feature_set_version);
        printf("Product type: %u\r\n", product_type);
    } else {
        printf("sgp30_get_feature_set_version failed!\r\n");
        return false;
    }

    uint64_t serial_id;
    err = sgp30_get_serial_id(&serial_id);
    if (err == STATUS_OK) {
        printf("SerialID: 0x%lx%lx\r\n", (uint32_t) ( ( serial_id & 0xFFFFFFFF00000000 ) > 32 ), (uint32_t) ( serial_id & 0x00000000FFFFFFFF ) );
    } else {
        printf("sgp30_get_serial_id failed!\n");
        return false;
    }

    /* Read gas raw signals */
    err = sgp30_measure_raw_blocking_read(&ethanol_raw_signal, &h2_raw_signal);
    if (err == STATUS_OK) {
        /* Print ethanol raw signal and h2 raw signal */
        printf("Ethanol raw signal: %u\n", ethanol_raw_signal);
        printf("H2 raw signal: %u\n", h2_raw_signal);
    } else {
        printf("error reading raw signals\n");
    }

    /* Consider the two cases (A) and (B):
     * (A) If no baseline is available or the most recent baseline is more than
     *     one week old, it must discarded. A new baseline is found with
     *     sgp30_iaq_init() */
    err = sgp30_iaq_init();
    if (err == STATUS_OK) {
        printf("sgp30_iaq_init done\r\n");
    } else {
        printf("sgp30_iaq_init failed!\r\n");
        return false;
    }
    /* (B) If a recent baseline is available, set it after sgp30_iaq_init() for
     * faster start-up */
    /* IMPLEMENT: retrieve iaq_baseline from presistent storage;
     * err = sgp30_set_iaq_baseline(iaq_baseline);
     */

    return true;
}

static uint16_t reading_count = 0;
bool get_voc_co2_eq( uint16_t *tvoc_ppb, uint16_t *co2_eq_ppm )
{
    uint16_t err;

    if( hs.valid_humidity_level == true ) {
        uint32_t ah = hs.current_humidity_level;   // absolute humidity in mg/m^3
        //  sgp30_set_absolute_humidity(ah);        // Enable once we have Pressure
    }

    err = sgp30_measure_iaq_blocking_read( tvoc_ppb, co2_eq_ppm );
    if( err == STATUS_OK) {
        PRINTF("tVOC  Concentration: %uppb\r\n", *tvoc_ppb);
        PRINTF("CO2eq Concentration: %uppm\r\n", *co2_eq_ppm);
    } else {
        printf("error reading IAQ values\n");
        return false;
    }

    /* Persist the current baseline every hour */
    if( ++reading_count % 3600 == 3599) {
        err = sgp30_get_iaq_baseline( &hs.iaq_baseline );
        if (err == STATUS_OK) {
            /* IMPLEMENT: store baseline to persistent storage */
        }
    }

    return true;
}
