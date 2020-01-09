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

/** @file sps30.c
 *
 *  Created on: October 27, 2019
 *      Author: greg.phillips
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "structs.h"
#include "sps30.h"
/******************************************************
 *                      Macros
 ******************************************************/
#ifdef PRINT_EV_DEBUGS_FOR_SENSORS
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

// const char *SPS_DRV_VERSION_STR = "Version 1.0 10-28-2019";
const char *SPS_DRV_VERSION_STR = "Version 1.1 11-13-2019";
/******************************************************
 *               Function Definitions
 ******************************************************/
/**
  * @brief Initialize the SPS30 Sensor
  * @param  None
  * @retval : None
  */
bool init_SPS30(void)
{
    struct sps30_measurement m;
    int16_t ret;

    /* Initialize I2C bus */
    sensirion_i2c_init();

    if( sps30_probe() != 0) {
        PRINTF("SPS sensor probing failed\n");
        return false;
    }

    PRINTF("SPS sensor probing successful\n");

    ret = sps30_start_measurement();
    if (ret < 0) {
        PRINTF("error starting Particulate measurement\n");
        return false;
    }
//    PRINTF("measurements started\n");

    sensirion_sleep_usec(SPS30_MEASUREMENT_DURATION_USEC); /* wait 1s */
    ret = sps30_read_measurement( &m );
    if (ret < 0) {
        PRINTF("error reading Particulate measurement: %d\n", ret );
        return false;
    } else {
        /*
        PRINTF("measured values:\n"
                "\t%0.2f pm1.0\n"
                "\t%0.2f pm2.5\n"
                "\t%0.2f pm4.0\n"
                "\t%0.2f pm10.0\n"
                "\t%0.2f nc0.5\n"
                "\t%0.2f nc1.0\n"
                "\t%0.2f nc2.5\n"
                "\t%0.2f nc4.5\n"
                "\t%0.2f nc10.0\n"
                "\t%0.2f typical particle size\n\n",
                m.mc_1p0, m.mc_2p5, m.mc_4p0, m.mc_10p0, m.nc_0p5, m.nc_1p0,
                m.nc_2p5, m.nc_4p0, m.nc_10p0, m.typical_particle_size);
                */
        ;
    }
    /*
     * Initialize the Kalman Filters
     */
    imx_kalman_filter_init( &hs.kf_PM_1_0, 6, 6, 0.2, m.mc_1p0 );
    imx_kalman_filter_init( &hs.kf_PM_2_5, 6, 6, 0.2, m.mc_2p5 );
    imx_kalman_filter_init( &hs.kf_PM_4_0, 6, 6, 0.2, m.mc_4p0 );
    imx_kalman_filter_init( &hs.kf_PM_10_0, 6, 6, 0.2, m.mc_10p0 );
    imx_kalman_filter_init( &hs.kf_no_PM_0_5, 6, 6, 0.2, m.nc_0p5 );
    imx_kalman_filter_init( &hs.kf_no_PM_1_0, 6, 6, 0.2, m.nc_1p0 );
    imx_kalman_filter_init( &hs.kf_no_PM_2_5, 6, 6, 0.2, m.nc_2p5 );
    imx_kalman_filter_init( &hs.kf_no_PM_4_0, 6, 6, 0.2, m.nc_4p0 );
    imx_kalman_filter_init( &hs.kf_no_PM_10_0, 6, 6, 0.2, m.nc_10p0 );
    imx_kalman_filter_init( &hs.kf_typical_particle, 6, 6, 0.2, m.typical_particle_size );
    return true;
}

bool start_particulate_matter_measurement(void)
{
    int16_t ret;

    ret = sps30_start_measurement();
    if (ret < 0) {
        return false;
    }
    return true;
}

bool update_particulate_matter(void)
{
    struct sps30_measurement m;
    int16_t ret;

    ret = sps30_read_measurement( &m );
    if (ret < 0) {
        return false;
    } else {
        hs.current_PM_1_0 = imx_kalman_filter_update_estimate( &hs.kf_PM_1_0, m.mc_1p0, 0 );
        hs.current_PM_2_5 = imx_kalman_filter_update_estimate( &hs.kf_PM_2_5, m.mc_2p5, 0  );
        hs.current_PM_4_0 = imx_kalman_filter_update_estimate( &hs.kf_PM_4_0, m.mc_4p0, 0  );
        hs.current_PM_10_0 = imx_kalman_filter_update_estimate( &hs.kf_PM_10_0, m.mc_10p0, 0  );
        hs.current_no_PM_0_5 = imx_kalman_filter_update_estimate( &hs.kf_no_PM_0_5, m.nc_0p5, 0  );
        hs.current_no_PM_1_0 = imx_kalman_filter_update_estimate( &hs.kf_no_PM_1_0, m.nc_1p0, 0  );
        hs.current_no_PM_2_5 = imx_kalman_filter_update_estimate( &hs.kf_no_PM_2_5, m.nc_2p5, 0  );
        hs.current_no_PM_4_0 = imx_kalman_filter_update_estimate( &hs.kf_no_PM_4_0, m.nc_4p0, 0  );
        hs.current_no_PM_10_0 = imx_kalman_filter_update_estimate( &hs.kf_no_PM_10_0, m.nc_10p0, 0  );
        hs.current_PM_index = 0;           // Calculate this value somehow
        hs.current_typical_particle = imx_kalman_filter_update_estimate( &hs.kf_typical_particle, m.typical_particle_size, 0 );
        hs.valid_particulate_matter = true;

    }
    return true;
}
