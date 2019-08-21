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
#include "deca_mac.h"
#include "eepromConfig.h"
#include "eeprom.h"
#include "string.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	uint8_t type;
	uint8_t anchor_id;
	float distance;
	int16 confidence;
} DistanceFrame;

typedef struct {
	unsigned long long t_rp;
	unsigned long long t_sr;
	unsigned long long t_rf;
} AnchorTimeStamps;

typedef struct {
	unsigned long long t_sp;
	unsigned long long t_rr;
	unsigned long long t_sf;
	unsigned long long t_ff;
} BeaconTimeStamps;

typedef enum RANGING_STATUS {
	RANGING_SUCCESS,
	SEND_POLL_FAILED,
	RECEIVE_POLL_FAILED,
	RECEIVE_RESPONSE_FAILED,
	SEND_FINAL_FAILED,
	RECEIVE_FINAL_FAILED,
	RECEIVE_TIMESTAMPS_FAILED
}RangingStatus;

typedef enum MESSAGE_TYPES {
	POLL,
	RESPONSE_INIT,
	SEND_FINAL,
	RESPONSE_DATA
}MessageType;


typedef enum TX_STATUS {
	TX_SUCCESS,
	TX_TIMEOUT
} TxStatus;

typedef enum RX_STATUS {
	RX_TIMEOUT,
	RX_DATA_FRAME_READY,
	RX_ERROR
} RxStatus;

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
RangingStatus range_with_anchor(uint8_t anchor_id, AnchorTimeStamps* a_stamps, BeaconTimeStamps* b_stamps, uint8* seq_num);
unsigned long long get_tx_timestamp(void);
unsigned long long get_rx_timestamp(void);
TxStatus transmit_frame(uint8* frame, int f_len, _Bool ranging);
int receive_frame(uint8* buffer, int max_len, int timeout);
int64_t get_tof(AnchorTimeStamps* a_stamps, BeaconTimeStamps* b_stamps);
double get_rx_power(dwt_rxdiag_t* diagnostics);
double get_fp_power(dwt_rxdiag_t* diagnostics);
double get_fp_snr(dwt_rxdiag_t* diagnostics);
int16 get_confidence(double rx_power, double fp_power, double snr);
void u_delay(int usec);
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
#define RANGING_LED_Pin GPIO_PIN_14
#define RANGING_LED_GPIO_Port GPIOB
#define USB_LED_Pin GPIO_PIN_15
#define USB_LED_GPIO_Port GPIOB
#define USB_PULLUP_Pin GPIO_PIN_6
#define USB_PULLUP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "port.h"
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
