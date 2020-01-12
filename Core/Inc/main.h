/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Enable_Comm_Processor_3V3_Pin GPIO_PIN_9
#define Enable_Comm_Processor_3V3_GPIO_Port GPIOB
#define SPI2_CLK___Not_Used_Pin GPIO_PIN_0
#define SPI2_CLK___Not_Used_GPIO_Port GPIOA
#define Debug_TX_Pin GPIO_PIN_2
#define Debug_TX_GPIO_Port GPIOA
#define Debug_RX_Pin GPIO_PIN_3
#define Debug_RX_GPIO_Port GPIOA
#define SPI_FLASH_CS_Pin GPIO_PIN_4
#define SPI_FLASH_CS_GPIO_Port GPIOA
#define SPI_SFLASH_CLK_Pin GPIO_PIN_5
#define SPI_SFLASH_CLK_GPIO_Port GPIOA
#define SPI_SFLASH_MISO_Pin GPIO_PIN_6
#define SPI_SFLASH_MISO_GPIO_Port GPIOA
#define SPI_SFLASH_MOSI_Pin GPIO_PIN_7
#define SPI_SFLASH_MOSI_GPIO_Port GPIOA
#define USB_C_in_1_Pin GPIO_PIN_0
#define USB_C_in_1_GPIO_Port GPIOB
#define USB_C_in_2_Pin GPIO_PIN_1
#define USB_C_in_2_GPIO_Port GPIOB
#define PMID___4_Pin GPIO_PIN_2
#define PMID___4_GPIO_Port GPIOB
#define Charge_INT___Pin GPIO_PIN_8
#define Charge_INT___GPIO_Port GPIOA
#define Power_Down___Pin GPIO_PIN_9
#define Power_Down___GPIO_Port GPIOA
#define Dead_Battery___Pin GPIO_PIN_6
#define Dead_Battery___GPIO_Port GPIOC
#define SPI2_OUT_LED_Display_Pin GPIO_PIN_10
#define SPI2_OUT_LED_Display_GPIO_Port GPIOA
#define UFP_Fault___Pin GPIO_PIN_15
#define UFP_Fault___GPIO_Port GPIOA
#define VSYS___2_Pin GPIO_PIN_3
#define VSYS___2_GPIO_Port GPIOB
#define LED_Array_Enable___Pin GPIO_PIN_4
#define LED_Array_Enable___GPIO_Port GPIOB
#define LED_Power_Enable_Pin GPIO_PIN_5
#define LED_Power_Enable_GPIO_Port GPIOB
#define TX_to_Comm_Processor_Pin GPIO_PIN_6
#define TX_to_Comm_Processor_GPIO_Port GPIOB
#define RX_from_Comm_Processor_Pin GPIO_PIN_7
#define RX_from_Comm_Processor_GPIO_Port GPIOB
#define Beeper_Pin GPIO_PIN_8
#define Beeper_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
