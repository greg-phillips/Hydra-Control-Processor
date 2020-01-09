/*
 * Copyright 2017, Sierra Telecom. All Rights Reserved.
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

/** @file .h
 *
 *  Created on: October 3, 2017
 *      Author: greg.phillips
 *
 */

#ifndef I2C_MANAGER_H_
#define I2C_MANAGER_H_

/*
 *	Defines for  i2c_manager
 *
 *	Manage interface to the i2c devices
 *
 *	These are accessed via a SPI-I2C bridge. Experience has shown this device locks up and needs to be reset frequently.
 *	This software dynamically adjusts the time required to access the i2c perhiperals to ensure that the device completes
 *	its transaction before the device is polled to check its status.
 *
 */

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define I2C_LTR_329ALS_ADDRESS  0x29    // External LUX Sensor LTR-329ALS
#define I2C_Si7020_ADDRESS      0x40    // Default for on board Temp / Humidity Sensor Si7020
#define I2C_SGP30_ADDRESS       0x58    // VOC CO2 Sensor
#define I2C_SPS30_ADDRESS       0x69    // Particulate Matter Sensor
#define I2C_BME680_ADDRESS      0x76    // Environmental Sensor
/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum {
    SENSOR_HUMIDITY,
    SENSOR_LUX,
    SENSOR_TEMPERATURE,
    SENSOR_CO2_VOC,
    SENSOR_PARTICULATE_MATTER,
    SENSOR_ENVIRONMENTAL,
} sensor_types_t;

typedef enum  {
    I2C_LTR_329ALS, // External LUX Sensor LTR-329ALS
    I2C_Si7020,     // Temperature / Humidity Sensor Si7020
    I2C_SGP30,      // VOC / CO2
    I2C_SPS30,      // Particulate Matter
    I2C_BME680,     // Bosch Environmental Sensor
    NO_I2C_DEVICES,
} i2c_devices_t;

typedef enum  {
    I2C_READ_COUNT = 0,
    I2C_WRITE_COUNT,
    F1_COUNT,
    F2_COUNT,
    F3_COUNT,
    F8_COUNT,
    F9_COUNT,
    NO_I2C_RESULT_CODES,
}i2c_result_codes_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
void i2c_init(void);
void i2c_manager_init( uint32_t current_time );
void i2c_sensor_process( uint32_t current_time );
bool i2c_write_sensor( i2c_devices_t i2c_device, uint8_t *tx_data, uint16_t tx_count );
bool i2c_write_read_sensor( i2c_devices_t i2c_device, uint8_t *tx_data, uint16_t tx_count, uint8_t *rx_data, uint16_t rx_count );
bool i2c_read_sensor( i2c_devices_t i2c_device, uint8_t *rx_data, uint16_t rx_count );
void print_sensor_process(void);
bool sensor_error( sensor_types_t sensor );
void reset_i2c_stats(void);
#endif /* I2C_MANAGER_H_ */
