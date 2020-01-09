/*
 * $Copyright 2017, Sierra Telecom, Inc.
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Sierra Telecom, Inc.;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Sierra Telecom, Inc.$
 */

/** @file
 *
 *  Si7020.h
 *
 *  Created on: September, 2017
 *      Author: greg.phillips
 *
 */
#ifndef I2C_Si7020_H_
#define I2C_Si7020_H_

/** @file
 *
 *  Si7020.h
 *  manage devices connected to the i2c bus
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
bool init_Si7020(void);
uint16_t Si7020_heater( uint16_t state );
uint16_t Si7020_get_serial( uint8_t serial_number[ 8 ] );
bool Si7020_get_temp(float *temp, float *humidity);

#endif /* I2C_Si7020_H_ */
