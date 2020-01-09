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
 *  LTR_329ALS.h
 *
 *  Created on: September, 2017
 *      Author: greg.phillips
 *
 */
#ifndef LTR_329ALS_H_
#define LTR_329ALS_H_

/** @file
 *
 *  i2c_LTR_329ALS.h
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
bool init_LTR_329ALS(void);
void LTR_32ALS_set_mode( uint16_t mode );
void LTR_329ALS_update_lux(void);
bool LTR_329ALS_get_lux_level( float *lux_level, float *ir_level );
void lux_test(void);

#endif /* LTR_329ALS_H_ */
