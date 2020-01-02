/*
 * Copyright 2018, Sierra Telecom. All Rights Reserved.
 *
 * This software, associated documentation and materials ("Software"),
 * is owned by Sierra Telecom ("Sierra") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Sierra
 * reserves the right to make changes to the Software without notice. Sierra
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Sierra does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Sierra product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Sierra's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Sierra against all liability.
 */

/** @file .c
 *
 *  Created on: October 14, 2019
 *      Author: greg.phillips
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "kalman_filter.h"

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
  * @brief  Initialize the Kalman Filter
  * @param  measured error, estimated error, q
  * error_measurement: Measurement Uncertainty - How much do we expect to our measurement vary
  * error_estitmate: Estimation Uncertainty - Can be initialized with the same value as e_mea since the kalman filter will adjust its value.
  * q: Process Variance - usually a small number between 0.001 and 1 - how fast your measurement moves. Recommended 0.01. Should be tunned to your needs.
  * @retval : None
  */
/*
 * Adapted from - https://github.com/denyssene/SimpleKalmanFilter
 * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 * Created by Denys Sene, January, 1, 2017.
 * Released under MIT License - see LICENSE file for details.
 */
void imx_kalman_filter_init( kalman_filter_t * kf, float error_measurement, float error_estimate, float q, float last_estimate )
{
  kf->error_measure = error_measurement;
  kf->error_estimate = error_estimate;
  kf->last_estimate = last_estimate;
  kf->q = q;
}

float imx_kalman_filter_update_estimate( kalman_filter_t *kf, float  measurement, float kalman_gain )
{
    float current_estimate;

    if( kalman_gain == 0 )  // Calculate or use provide value
        kalman_gain = kf->error_estimate / ( kf->error_estimate + kf->error_measure );
    current_estimate = kf->last_estimate + ( kalman_gain * ( measurement - kf->last_estimate) );
    kf->error_estimate =  ( ( 1.0 - kalman_gain ) * kf->error_estimate ) + ( fabs( kf->last_estimate - current_estimate) * kf->q );
    kf->last_estimate = current_estimate;

  return current_estimate;
}
