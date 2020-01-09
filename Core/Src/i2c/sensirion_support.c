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

/** @file sensirion_support.c
 *
 *  Created on: April 25, 2019
 *      Author: greg.phillips
 *
 *  Hardware support functions needed for the sensirion source lib
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32g0xx_hal.h"


#include "i2c_manager.h"
#include "sensirion_common.h"
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
  * @brief sleep some uS
  * @param  micro seconds to sleep
  * @retval : None
  */
void sensirion_sleep_usec(uint32_t useconds)
{
    HAL_Delay( useconds / 1000 );
}
/**
  * @brief setup I2C
  * @param  None
  * @retval : None
  */
void sensirion_i2c_init(void)
{
    /*
     * This is handled automatically in i2c handler code
     */
}
/**
  * @brief Read from I2C bus
  * @param  Address, pointer to data, count of data
  * @retval : True/False
  */
uint8_t sensirion_i2c_read(uint8_t address, uint8_t* data, uint16_t count)
{

    if( address == I2C_SGP30_ADDRESS ) {
        if( i2c_read_sensor( I2C_SGP30, data, count ) == true )
            return STATUS_OK;
        else
            return STATUS_FAIL;
    } else if( address == I2C_SPS30_ADDRESS ) {
        if( i2c_read_sensor( I2C_SPS30, data, count ) == true )
            return STATUS_OK;
        else
            return STATUS_FAIL;
    } else
        return STATUS_FAIL;
}
/**
  * @brief Read from I2C bus
  * @param  Address, pointer to data, count of data
  * @retval : True/False
  */
uint8_t sensirion_i2c_write(uint8_t address, const uint8_t* data, uint16_t count)
{

    if( address == I2C_SGP30_ADDRESS ) {
        if( i2c_write_sensor( I2C_SGP30, (uint8_t *) data, count ) == true )
            return STATUS_OK;
        else
            return STATUS_FAIL;
    } else if( address == I2C_SPS30_ADDRESS ) {
        if( i2c_write_sensor( I2C_SPS30, (uint8_t *) data, count ) == true )
            return STATUS_OK;
        else
            return STATUS_FAIL;
    } else
        return STATUS_FAIL;
}
