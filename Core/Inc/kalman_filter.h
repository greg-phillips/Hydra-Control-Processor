/*
 * Copyright 2019, Sierra Telecom. All Rights Reserved.
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

/** @file kalman_filter.h
 *
 *  Created on: October 14, 2019
 *      Author: greg.phillips
 *
 */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

/*
 *	Defines for Kalman Filter
 *
 */

/******************************************************
 *                  Macros
 ******************************************************/

/******************************************************
 *                  Constants
 ******************************************************/
#define DEFAULT_Q_VALUE     0.2
/******************************************************
 *                  Enumerations
 ******************************************************/

/******************************************************
 *                  Type Definitions
 ******************************************************/
typedef struct kalman_filter {
    float error_measure;
    float error_estimate;
    float q;
    float last_estimate;
    float kalman_gain;
} kalman_filter_t;
/******************************************************
 *                  Structures
 ******************************************************/

/******************************************************
 *                  Function Declarations
 ******************************************************/
void imx_kalman_filter_init( kalman_filter_t * kf, float error_measurement, float error_estimate, float q, float last_estimate );
float imx_kalman_filter_update_estimate( kalman_filter_t * kf, float  measurement, float kalman_gain );

#endif /* KALMAN_FILTER_H_ */
