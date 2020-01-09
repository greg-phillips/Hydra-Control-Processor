/*
* $Copyright 2017, Sierra Telecom, Inc.
* All Rights Reserved.
*
* This is UNPUBLISHED PROPRIETARY SOURCE CODE of Sierra Telecom, Inc.;
* the contents of this file may not be disclosed to third parties, copied
* or duplicated in any form, in whole or in part, without the prior
* written permission of Sierra Telecom, Inc.
*/

/* @file product.h
*
* Created on: Oct 27, 2019
* Author: Auto Generated Code Do NOT Modify
*
* Product: Environmental Gateway uses the ISM43340 Platform
* The following controls and sensors
* Controls
* Sensors
*   0 - Host S/W Revision
*   1 - RF Scan
*   2 - Variable Data In
*   3 - WiFi BSSID
*   4 - WiFi Channel
*   5 - WiFi RF Noise
*   6 - WiFi RSSI
*   7 - BLE Device RSSI Levels
*   8 - CO2 Equivalent
*   9 - Humidity
*   10 - Mass Concentration PM 1.0
*   11 - Mass Concentration PM 10.0
*   12 - Mass Concentration PM 2.5
*   13 - Mass Concentration PM 4.0
*   14 - Number Concentration PM 0.5
*   15 - Number Concentration PM 1.0
*   16 - Number Concentration PM 10.0
*   17 - Number Concentration PM 2.5
*   18 - Number Concentration PM 4.0
*   19 - Particulate Matter Index
*   20 - Temperature
*   21 - Typical Particle Size
*   22 - VOC
*/

#ifndef IMATRIX_PRODUCT_H_
#define IMATRIX_PRODUCT_H_
/******************************************************
*                      Macros
******************************************************/

/******************************************************
*                    Constants
******************************************************/
/*
* Memory Use Definitions
*/
#define IMATRIX_HISTORY_SIZE        ( 10 )

#define IMX_LOCAL_SECONDS_FROM_UTC  -28800      // PST USA

/*
* Product and Operating details
*/
#define IMX_MANUFACTUER_ID          1583546208
#define IMX_PRODUCT_ID              0x7ec66bf

/*
* Access Settings
*/
#define IMX_CLI_ENABLED             true

/*
* Define Control IDs for Controls
*/
/*
* Integrated Controls
*/
#define IMX_NO_INTEGRATED_CONTROLS ( 0 )

/*
* Product Controls
*/
#define IMX_NO_PRODUCT_CONTROLS ( 0 )

#define IMX_NO_CONTROLS ( IMX_NO_INTEGRATED_CONTROLS + IMX_NO_PRODUCT_CONTROLS)


/*
* Define Sensor IDs for Sensors
*/
/*
* Integrated Sensors
*/
#define IMX_HOST_S_W_REVISION_103378620 0x6296ebc
#define IMX_RF_SCAN_123455660 0x75bc8ac
#define IMX_VARIABLE_DATA_IN_2058836870 0x7ab75b86
#define IMX_WIFI_BSSID_173116155 0xa518afb
#define IMX_WIFI_CHANNEL_173916172 0xa5dc00c
#define IMX_WIFI_RF_NOISE_1248909411 0x4a70d863
#define IMX_WIFI_RSSI_101600364 0x60e4c6c
#define IMX_NO_INTEGRATED_SENSORS  ( 7 )

/*
* Product Sensors
*/
#define BLE_DEVICE_RSSI_LEVELS_2037468041 0x79714b89
#define CO2_EQUIVALENT_2088797904 0x7c8086d0
#define HUMIDITY_342048076 0x14633d4c
#define MASS_CONCENTRATION_PM_1_0_32791331 0x1f45b23
#define MASS_CONCENTRATION_PM_10_0_734215216 0x2bc33c30
#define MASS_CONCENTRATION_PM_2_5_1698993330 0x654494b2
#define MASS_CONCENTRATION_PM_4_0_1858953557 0x6ecd6155
#define NUMBER_CONCENTRATION_PM_0_5_1642774029 0x61eabe0d
#define NUMBER_CONCENTRATION_PM_1_0_1984590476 0x764a728c
#define NUMBER_CONCENTRATION_PM_10_0_1909326236 0x71ce019c
#define NUMBER_CONCENTRATION_PM_2_5_1751774023 0x6869f347
#define NUMBER_CONCENTRATION_PM_4_0_891153645 0x351deced
#define PARTICULATE_MATTER_INDEX_1695740845 0x6512f3ad
#define TEMPERATURE_1913614964 0x720f7274
#define TYPICAL_PARTICLE_SIZE_159299661 0x97eb84d
#define VOC_1189979747 0x46eda663
#define IMX_NO_PRODUCT_SENSORS  ( 16 )

#define SCB_IMX_HOST_S_W_REVISION_103378620 ( 0 )
#define SCB_IMX_RF_SCAN_123455660 ( 1 )
#define SCB_IMX_VARIABLE_DATA_IN_2058836870 ( 2 )
#define SCB_IMX_WIFI_BSSID_173116155 ( 3 )
#define SCB_IMX_WIFI_CHANNEL_173916172 ( 4 )
#define SCB_IMX_WIFI_RF_NOISE_1248909411 ( 5 )
#define SCB_IMX_WIFI_RSSI_101600364 ( 6 )
#define SCB_BLE_DEVICE_RSSI_LEVELS_2037468041 ( 7 )
#define SCB_CO2_EQUIVALENT_2088797904 ( 8 )
#define SCB_HUMIDITY_342048076 ( 9 )
#define SCB_MASS_CONCENTRATION_PM_1_0_32791331 ( 10 )
#define SCB_MASS_CONCENTRATION_PM_10_0_734215216 ( 11 )
#define SCB_MASS_CONCENTRATION_PM_2_5_1698993330 ( 12 )
#define SCB_MASS_CONCENTRATION_PM_4_0_1858953557 ( 13 )
#define SCB_NUMBER_CONCENTRATION_PM_0_5_1642774029 ( 14 )
#define SCB_NUMBER_CONCENTRATION_PM_1_0_1984590476 ( 15 )
#define SCB_NUMBER_CONCENTRATION_PM_10_0_1909326236 ( 16 )
#define SCB_NUMBER_CONCENTRATION_PM_2_5_1751774023 ( 17 )
#define SCB_NUMBER_CONCENTRATION_PM_4_0_891153645 ( 18 )
#define SCB_PARTICULATE_MATTER_INDEX_1695740845 ( 19 )
#define SCB_TEMPERATURE_1913614964 ( 20 )
#define SCB_TYPICAL_PARTICLE_SIZE_159299661 ( 21 )
#define SCB_VOC_1189979747 ( 22 )
#define IMX_NO_SENSORS ( IMX_NO_INTEGRATED_SENSORS + IMX_NO_PRODUCT_SENSORS)

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

#endif /* IMATRIX_PRODUCT_H_ */

/* [] END OF FILE */
