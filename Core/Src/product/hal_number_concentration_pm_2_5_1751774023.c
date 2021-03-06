/*
* $Copyright 2013-2017, Sierra Telecom, Inc.
* All Rights Reserved.
*
* This is UNPUBLISHED PROPRIETARY SOURCE CODE of Sierra Telecom, Inc.;
* the contents of this file may not be disclosed to third parties, copied
* or duplicated in any form, in whole or in part, without the prior
* written permission of Sierra Telecom, Inc.$
*/

/*
*  Created on: Oct 27, 2019
*  Author: Auto Generated Code Do NOT Modify
*/
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <memory.h>

#include "structs.h"
#include "i2c_manager.h"
#include "product.h"
#include "hal_functions.h"

/**@file
*
*    hal_number_concentration_pm_2_5_1751774023.c
*
*    to be included in hal_functions.c
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
*               Function Declarations
******************************************************/

/******************************************************
*               Variable Definitions
******************************************************/
extern hydra_status_t hs;

/******************************************************
*               Function Definitions
******************************************************/
/**
* @brief initialize the hardware for temperature sensing
* @param  None
* @retval : None
*/

void init_number_concentration_pm_2_5_1751774023(uint16_t arg)
{
}

/**
* @brief sample the temperature
* @param  None
* @retval : None
*/
imx_result_t sample_number_concentration_pm_2_5_1751774023(uint16_t arg, void *value )
{
    if( sensor_error( SENSOR_PARTICULATE_MATTER ) == true )
        return IMX_SENSOR_ERROR;
    if( hs.valid_particulate_matter == true ) {
        memcpy( value, &hs.current_PM_2_5, IMX_SAMPLE_LENGTH );
        return IMX_SUCCESS;
    } else
        return IMX_NO_DATA;
}
