/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <inttypes.h>
#include "deca_types.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "deca_mac.h"
#include "port.h"
#include "eepromConfig.h"
#include "eeprom.h"
#include "string.h"
#include "usb_device.h"
//#include "usbd_cdc_if.h"
#include "rtls.h"
#include "comm.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


enum CAN_FRAME_TYPES {
	DISTANCE_FRAME_TYPE = 0x00,
	ERROR_FRAME_TYPE = 0xFF
};

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
void debug(UART_HandleTypeDef* huart, char* text);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TXLED_Pin GPIO_PIN_1
#define TXLED_GPIO_Port GPIOA
#define RXOKLED_Pin GPIO_PIN_4
#define RXOKLED_GPIO_Port GPIOA
#define DW_NSS_Pin GPIO_PIN_0
#define DW_NSS_GPIO_Port GPIOB
#define DW_RESET_Pin GPIO_PIN_1
#define DW_RESET_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define DW_IRQn_Pin GPIO_PIN_10
#define DW_IRQn_GPIO_Port GPIOB
#define DW_IRQn_EXTI_IRQn EXTI15_10_IRQn
#define USB_RX_LED_Pin GPIO_PIN_11
#define USB_RX_LED_GPIO_Port GPIOB
#define RANGING_LED_Pin GPIO_PIN_14
#define RANGING_LED_GPIO_Port GPIOB
#define USB_TX_LED_Pin GPIO_PIN_15
#define USB_TX_LED_GPIO_Port GPIOB
#define USB_PULLUP_Pin GPIO_PIN_6
#define USB_PULLUP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define DW_IRQn_Type EXTI15_10_IRQn
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
