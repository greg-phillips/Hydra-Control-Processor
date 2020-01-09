/*
* $Copyright 2017, Sierra Telecom, Inc.
* All Rights Reserved.
*
* This is UNPUBLISHED PROPRIETARY SOURCE CODE of Sierra Telecom, Inc.;
* the contents of this file may not be disclosed to third parties, copied
* or duplicated in any form, in whole or in part, without the prior
* written permission of Sierra Telecom, Inc.$
*/

/*
*  Created on: Jan 22, 2017
*  Author:
*/

#ifndef HAL_TEMPERATURE_1913614964_
#define HAL_TEMPERATURE_1913614964_

/** @file hal_temperature_1913614964.h
*
*    define functions for hal_temperature_1913614964.c
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
void init_temperature_1913614964(uint16_t arg);
imx_result_t sample_temperature_1913614964(uint16_t arg, void *value );
#endif /* HAL_TEMP_H_ */