/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* CAN ID, change to desired value if flashing for the first time */
#define SELF_CAN_ID 		0x01

#ifdef ANCHOR
/* ID of this device, change to desired value if flashing for the first time */
#define SELF_ID 			0x11
#endif

#ifdef BEACON
/* ID of this device will be the CAN bus id, because why not */
#define SELF_ID 			SELF_CAN_ID
#endif

#define IF_TYPE_USB 	0x01
#define IF_TYPE_UART 	0x02

/* RX Timeout in milliseconds */
#define RX_TIMEOUT 			50

/* Broadcast period in milliseconds */
#define ANCHOR_BROADCAST_PERIOD 250

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char debug_buf[512];

static dwt_config_t config = {
		2,               /* Channel number. */
		DWT_PRF_64M,     /* Pulse repetition frequency. */
		DWT_PLEN_128,   /* Preamble length. Used in TX only. */
		DWT_PAC8,       /* Preamble acquisition chunk size. Used in RX only. */
		9,               /* TX preamble code. Used in TX only. */
		9,               /* RX preamble code. Used in RX only. */
		0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
		DWT_BR_6M8,      /* Data rate. */
		DWT_PHRMODE_STD, /* PHY header mode. */
		(1025 + 64)    	 /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

DistanceFrame anchors[NUMBER_OF_ANCHORS];

state_data_t 	state_data;
state_t 		state 		= IDLE;
uint32_t timeout_start 		= 0;
_Bool 			new_frame 	= 0;
RangingStatus 	ranging_status;
dwt_rxdiag_t 	rx_diagnostics;
double 			rx_power;
double 			fp_power;
double 			snr;
_Bool 			do_ranging 					= 0;
_Bool 			dwm1000_setup_success 		= 0;
BeaconTimeStamps beacon_stamps;
uint8 			sequence_num 				= 0;
uint32_t 		operating_mode 				= DEVICE_MODE_TAG;

unsigned long long t_sp, t_rr, t_sf, t_rp, t_sr, t_rf;

int size = 0; 						// used for sprintf and debugging
uint8_t uart_buf[INPUT_BUFFER_SIZE] = {}; 		// store data received over UART2
HAL_StatusTypeDef 	UART_status;
HAL_StatusTypeDef 	uart2_status; 	// status of uart2
int8_t 				usb_status;
uint32_t 			bytes_read; 	//
extern AnchorData* anchor_data; 	// place to store data for anchors (defined in comm.h)
CONFIG_FIELD_TYPE self_config[FIELD_SIZE*NUM_FIELDS]; 	// configuration for ourselves
uint8_t packet[INPUT_BUFFER_SIZE]; 	// will hold received packet from host
uint8_t packet_type = 0; 			// type of received packet
int packet_len 	= 0; 			// length of received packet
int packet_rcvd 	= 0; 			// indicate if packet has been received and how to process it
uint8_t send_buf[SEND_BUF_SIZE]; 	// lorge buffer for response packet
int usb_ind = 0;
uint16 src_address;
uint16 src_panid;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void init_from_fields(void* fields, dwt_config_t* dw_config);
void init_from_config(CONFIG_FIELD_TYPE* config, dwt_config_t* dw_config);
void set_state(state_data_t* sd, state_t state);
void set_state_idle(state_data_t* state);
void set_state_tag_idle(state_data_t* sd);
void set_state_anchor_idle(state_data_t* state);
void set_wait_for_poll(state_data_t* sd);
void set_wait_for_final(state_data_t* sd);
void set_wait_for_repsonse(state_data_t* sd);
void set_wait_for_data(state_data_t* sd);
void set_state_process_rx_frame(state_data_t* state);
void process_anchor_broadcast(state_data_t* state);
void tag_wait_timeout(state_data_t* state);
void anchor_wait_timeout(state_data_t* state);
void next_anchor(state_data_t* state);
int  check_rx_state();
void disable_ranging(state_data_t* state);
void enable_ranging(state_data_t* state);
void dw_it_disable(void);
void dw_it_enable(void);
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

	anchor_data = (AnchorData*)malloc((ANCHOR_LIST_SIZE + 1)*sizeof(AnchorData));
	init_anchor_array(anchor_data, ANCHOR_LIST_SIZE + 1);

	state_data.anchors = anchor_data;

	init_field_memory(self_config); // set everything to defaults

	read_config_from_eeprom(self_config); 	// pull any existing values from flash

	init_from_config(self_config, &config); // inititalize the DWM1000 from our fields

//	HAL_Delay(state_data.self_id * 10); // delay to hopefully stagger broadcast messages

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// ---- check UART for data ----
		uart2_status = HAL_UART_Receive(&huart2, uart_buf, HDDR_LEN, 5); //INPUT_TIMEOUT);
		if((packet_type = uart_buf[0]) != 0 && uart2_status == HAL_OK){
			HAL_GPIO_WritePin(USB_RX_LED_GPIO_Port, USB_RX_LED_Pin, GPIO_PIN_SET);
			packet_len = HDDR_LEN + uart_buf[1] + 1; 	// get the packet length

			if(packet_len > INPUT_BUFFER_SIZE)
				packet_len = INPUT_BUFFER_SIZE; // clamp packet_len just in case

			// read the packet
			uart2_status = HAL_UART_Receive(&huart2, uart_buf + HDDR_LEN, uart_buf[1] + 1, INPUT_TIMEOUT);

			// if all is well
			if(uart2_status == HAL_OK && uart_buf[packet_len - 1] == STOP_BYTE){
				packet_rcvd = IF_TYPE_UART;
				memcpy(packet, uart_buf, packet_len);
			}

			memset(uart_buf, 0, packet_len); // clear the packet buffer
			HAL_GPIO_WritePin(USB_RX_LED_GPIO_Port, USB_RX_LED_Pin, GPIO_PIN_RESET);

		}else if((packet_type = usb_rx_buffer[0]) != 0){
			HAL_GPIO_WritePin(USB_RX_LED_GPIO_Port, USB_RX_LED_Pin, GPIO_PIN_SET);
			packet_len 	= usb_rx_buffer[1] + HDDR_LEN + 1;

			if(packet_len > INPUT_BUFFER_SIZE)
				packet_len = INPUT_BUFFER_SIZE; // clamp packet_len just in case

			if(usb_rx_buffer[packet_len - 1] == STOP_BYTE){
				packet_rcvd = IF_TYPE_USB;
				memcpy(packet, usb_rx_buffer, packet_len);
			}

			memset(usb_rx_buffer, 0, packet_len);

			HAL_GPIO_WritePin(USB_RX_LED_GPIO_Port, USB_RX_LED_Pin, GPIO_PIN_RESET);
		}

		// ---- Process received packet ----
		if(packet_rcvd > 0){
			debug(&huart1, "Processing packet\r\n");
			int packet_status = process_packet(packet, send_buf, self_config, &state_data);

			if(packet_status == PACKET_OK){
				if(packet[0] == CMD_SET_CONFIG){
					init_from_config(self_config, &config); // re-initialize with the new parameters
					save_fields_to_eeprom(self_config);
				}
			}else{
				sprintf(debug_buf, "Packet error: %d\r\n", packet_status);
				debug(&huart1, debug_buf);
			}
			int send_size = HDDR_LEN + send_buf[1] + 1;
			switch(packet_rcvd){
			case IF_TYPE_UART:
				HAL_GPIO_WritePin(GPIOB, USB_TX_LED_Pin, GPIO_PIN_SET);
				uart2_status = HAL_UART_Transmit(&huart2, send_buf, send_size, INPUT_TIMEOUT);
//				uart2_status = HAL_UART_Transmit_IT(&huart2, send_buf, send_size);
				HAL_GPIO_WritePin(GPIOB, USB_TX_LED_Pin, GPIO_PIN_RESET);
				break;
			case IF_TYPE_USB:
				HAL_GPIO_WritePin(GPIOB, USB_TX_LED_Pin, GPIO_PIN_SET);
				CDC_Transmit_FS(send_buf, send_size);
				HAL_GPIO_WritePin(GPIOB, USB_TX_LED_Pin, GPIO_PIN_RESET);
				break;
			default:
				break;
			}
			packet_rcvd = 0; // reset this flag
		}

		// ==== STATE MACHINE ====
		if(state_data.mode == DEVICE_MODE_TAG){
			switch(state_data.state){
			case IDLE:
				//debug(&huart1, "idle...\r");
				// check for the trigger signal
				if(state_data.ranging){

					dwt_forcetrxoff(); // in case we were in RX mode looking for a beacon frame

					// select whichever anchor we're ranging with
					state_data.transact_id = state_data.anchors[state_data.anchor_ind].id;
					size = sprintf(debug_buf, "(Anchor index: %d) Sending POLL to anchor %d\r\n", state_data.anchor_ind, state_data.transact_id);
					debug(&huart1, debug_buf);

					// LED on
					//HAL_GPIO_WritePin(RANGING_LED_GPIO_Port, RANGING_LED_Pin, GPIO_PIN_SET);
					int data_len = rtls_make_mac_header(&state_data, FRAME_TYPE_DATA); //make_mac_header(state_data.tx_buffer, state_data.transact_id, state_data.transact_id, state_data.seq_num);
					state_data.tx_buffer[data_len++] = POLL; // right after the MAC header

					if(transmit_frame(state_data.tx_buffer, data_len + 2, 1) != TX_SUCCESS)
						break;

					state_data.t_sp = get_tx_timestamp();
					set_wait_for_repsonse(&state_data);
				}else if(HAL_GetTick() - state_data.timer_start >= state_data.ranging_period &&
						state_data.num_anchors > 0){

					state_data.timer_start = HAL_GetTick();
					enable_ranging(&state_data);
				}
				break;
			case WAIT_FOR_RESPONSE:
				if(state_data.new_frame){
					state_data.new_frame = 0;
					// check the frame type and source address
					if(state_data.rx_buffer[MAC_SIZE_EXPECTED] == RESPONSE_INIT &&
							get_src_addr(state_data.rx_buffer) == state_data.transact_id){

						size = sprintf(debug_buf, "RESPONSE_INIT from %d\r\n", state_data.transact_id);
						debug(&huart1, debug_buf);

						state_data.t_rr = get_rx_timestamp();

						send_response_final(&state_data);

						state_data.t_sf = get_tx_timestamp();

						set_wait_for_data(&state_data);
					}else{
						dwt_rxenable(DWT_START_RX_IMMEDIATE); // re-enable receiver and keep listening
					}
				}else if(HAL_GetTick() - state_data.timer_start >= RX_TIMEOUT){
					tag_wait_timeout(&state_data);
				}
				break;
			case WAIT_FOR_DATA:
				if(state_data.new_frame){
					state_data.new_frame = 0;
					// check the frame type and source address
					if(state_data.rx_buffer[MAC_SIZE_EXPECTED] == RESPONSE_DATA &&
							get_src_addr(state_data.rx_buffer) == state_data.transact_id){

						size = sprintf(debug_buf, "RESPONSE_DATA from %d\r\n", state_data.transact_id);
						debug(&huart1, debug_buf);

						AnchorData* anchor = &(state_data.anchors[state_data.anchor_ind]);

						// COPY ALL THE DATA OVER
						int ind = MAC_SIZE_EXPECTED + 1; // index for start of timestamps
						memcpy(&state_data.t_rp, state_data.rx_buffer + ind, sizeof(state_data.t_rp));
						ind += sizeof(state_data.t_rp);
						memcpy(&state_data.t_sr, state_data.rx_buffer + ind, sizeof(state_data.t_sr));
						ind += sizeof(state_data.t_sr);
						memcpy(&state_data.t_rf, state_data.rx_buffer + ind, sizeof(state_data.t_rf));
						ind += sizeof(state_data.t_rf);
						memcpy(&anchor->x, state_data.rx_buffer + ind, sizeof(anchor->x));
						ind += sizeof(anchor->x);
						memcpy(&anchor->y, state_data.rx_buffer + ind, sizeof(anchor->y));
						ind += sizeof(anchor->y);
						memcpy(&anchor->z, state_data.rx_buffer + ind, sizeof(anchor->z));
						ind += sizeof(anchor->z);
						anchor->timestamp = HAL_GetTick();

						get_tof(&state_data);

						state_data.distance = DISTANCE_FROM_TOF(state_data.tof);
						//state_data.ranging = 0; // done ranging

						anchor->timeout_count = 0;
						anchor->distance = state_data.distance;

						HAL_GPIO_WritePin(RANGING_LED_GPIO_Port, RANGING_LED_Pin, GPIO_PIN_RESET);

						disable_ranging(&state_data);

						set_state_tag_idle(&state_data); 		// back to idle state

						next_anchor(&state_data); 				// increment anchor index

						//HAL_Delay(state_data.ranging_period); 	// take a break

					}else{
						dwt_rxenable(DWT_START_RX_IMMEDIATE); 	// re-enable receiver and keep listening
					}
				}else if(HAL_GetTick() - state_data.timer_start >= RX_TIMEOUT){
					tag_wait_timeout(&state_data);
				}
				break;
			default:
				set_state_tag_idle(&state_data);
				break;
			}
		}else if(state_data.mode == DEVICE_MODE_ANCHOR){
			switch(state_data.state){
			case IDLE:
				set_wait_for_poll(&state_data);
				break;
			case WAIT_FOR_POLL:
				if(state_data.new_frame){
					state_data.new_frame = 0; // deassert new_frame
					debug(&huart1, "new frame\r\n");
					// if we're NOT ranging already, and this is a POLL frame
					if(!state_data.ranging && state_data.rx_buffer[MAC_SIZE_EXPECTED] == POLL){

						enable_ranging(&state_data); // indicate ranging in progress

						state_data.t_rp = get_rx_timestamp(); 		// stash rx timestamp
						//state_data.transact_id = get_src_addr(state_data.rx_buffer); // indicate who we're talking with

						size = sprintf(debug_buf, "Got POLL from %d\r\n", state_data.transact_id);
						debug(&huart1, debug_buf);

						send_response_init(&state_data); 	// send a RESPONSE_INIT frame & stash timestamp
						if(state_data.tx_status != TX_SUCCESS){
							init_from_config(self_config, &config); // just reinitialize
							set_state_idle(&state_data);
						}else{
							set_wait_for_final(&state_data); 	// transition to state WAIT_FOR_FINAL
						}
					}
//					else if(state_data.rx_buffer[MAC_SIZE_EXPECTED] == ANCHOR_BROADCAST){
//						// we've received an anchor broadcast signal
//						//HAL_Delay(state_data.self_id * 10); // delay to stagger broadcast transmission;
//						dwt_rxenable(DWT_START_RX_IMMEDIATE);
//					}
					else{
						dwt_rxenable(DWT_START_RX_IMMEDIATE); // re-enable receiver and keep listening
					}
				}
				else if(HAL_GetTick() - state_data.broadcast_timer >= ANCHOR_BROADCAST_PERIOD){
					// BROADCAST
					disable_ranging(&state_data);
					debug(&huart1, "Broadcasting our info...\r\n");
					dwt_forcetrxoff();

					send_anchor_broadcast(&state_data);
					set_wait_for_poll(&state_data); // enables receiver and resets timer_start
				}
				break;
			case WAIT_FOR_FINAL:
				if(state_data.new_frame){
					state_data.new_frame = 0;
					// if it's a SEND_FINAL frame and the SRC ADDR matches the current transact_id
					if(state_data.rx_buffer[MAC_SIZE_EXPECTED] == SEND_FINAL &&
							state_data.transact_id == get_src_addr(state_data.rx_buffer)){

						size = sprintf(debug_buf, "Got SEND_FINAL from %d\r\n", state_data.transact_id);
						debug(&huart1, debug_buf);

						state_data.t_rf = get_rx_timestamp(); 	// stash the timestamp

						send_response_data(&state_data); 		// send a RESPONSE_DATA frame

						disable_ranging(&state_data);

						set_wait_for_poll(&state_data); // return to WAIT_FOR_POLL
					}else{
						dwt_rxenable(DWT_START_RX_IMMEDIATE);
					}
				}else if(HAL_GetTick() - state_data.timer_start >= RX_TIMEOUT){
					anchor_wait_timeout(&state_data);
				}
				break;
			default:
				set_wait_for_poll(&state_data);
				break;
			}
		}else{
			// unrecognized operating mode
		}

	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DW_NSS_Pin|DW_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, USB_RX_LED_Pin|RANGING_LED_Pin|USB_TX_LED_Pin|USB_PULLUP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TXLED_Pin RXOKLED_Pin */
  GPIO_InitStruct.Pin = TXLED_Pin|RXOKLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DW_NSS_Pin USB_RX_LED_Pin RANGING_LED_Pin USB_TX_LED_Pin 
                           USB_PULLUP_Pin */
  GPIO_InitStruct.Pin = DW_NSS_Pin|USB_RX_LED_Pin|RANGING_LED_Pin|USB_TX_LED_Pin 
                          |USB_PULLUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DW_RESET_Pin */
  GPIO_InitStruct.Pin = DW_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DW_IRQn_Pin */
  GPIO_InitStruct.Pin = DW_IRQn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DW_IRQn_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(DW_IRQn_Type, 0, 0);
  HAL_NVIC_EnableIRQ(DW_IRQn_Type);

}

/* USER CODE BEGIN 4 */
void set_state(state_data_t* sd, state_t state){
	sd->last_state = sd->state;
	sd->state = state;
}

void set_state_idle(state_data_t* state){
	if(state->mode == DEVICE_MODE_TAG){
		set_state_tag_idle(state);
	}else{
		set_state_anchor_idle(state);
	}
}
void set_state_tag_idle(state_data_t* sd){
	set_state(sd, IDLE); //sd->state = IDLE;
	sd->timer_start = HAL_GetTick(); 	// reset the timer
	disable_ranging(sd);
	dwt_rxenable(DWT_START_RX_IMMEDIATE); 	// in case there are any anchor beacon frames
}

void set_state_anchor_idle(state_data_t* state){
	disable_ranging(state);
	set_state(state, IDLE); //state->state = IDLE;
}

void set_wait_for_poll(state_data_t* sd){
	set_state(sd, WAIT_FOR_POLL); //sd->state = WAIT_FOR_POLL;
	sd->timer_start = HAL_GetTick(); 	// reset the timer
	sd->broadcast_timer = HAL_GetTick();
	dwt_rxenable(DWT_START_RX_IMMEDIATE); /* Activate reception immediately. */
}

void set_wait_for_final(state_data_t* sd){
	set_state(sd, WAIT_FOR_FINAL); //sd->state = WAIT_FOR_FINAL;
	sd->timer_start = HAL_GetTick(); 	// reset the timer
	dwt_rxenable(DWT_START_RX_IMMEDIATE); /* Activate reception immediately. */
}

void set_wait_for_repsonse(state_data_t* sd){
	set_state(sd, WAIT_FOR_RESPONSE); //sd->state = WAIT_FOR_RESPONSE;
	sd->timer_start = HAL_GetTick(); 	// reset the timer
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void process_anchor_broadcast(state_data_t* state){
	if(state->mode == DEVICE_MODE_TAG){
		// add the new anchor
		add_new_anchor(state, get_anchor_from_frame(state->rx_buffer));
		//enable_ranging(state);
	}
	state->new_frame = 0;
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void disable_ranging(state_data_t* state){
	debug(&huart1, "Disabling ranging\r\n");
	state->ranging = 0;
	HAL_GPIO_WritePin(RANGING_LED_GPIO_Port, RANGING_LED_Pin, GPIO_PIN_RESET);
}
void enable_ranging(state_data_t* state){
	debug(&huart1, "Enabling ranging\r\n");
	state->ranging = 1;
	HAL_GPIO_WritePin(RANGING_LED_GPIO_Port, RANGING_LED_Pin, GPIO_PIN_SET);
}

void set_wait_for_data(state_data_t* sd){
	set_state(sd, WAIT_FOR_DATA); //sd->state = WAIT_FOR_DATA;
	sd->timer_start = HAL_GetTick(); 	// reset the timer
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void tag_wait_timeout(state_data_t* state){
	check_rx_state(); // check for and clear any errors
	state->anchors[state->anchor_ind].timeout_count++;
	size = sprintf(debug_buf, "(Anchor ind: %d) timed out waiting for %d\r\n", state->anchor_ind, state->anchors[state->anchor_ind].id);
	debug(&huart1, debug_buf);

	// have we timed out 3 times in a row
//	if(state->anchors[state->anchor_ind].timeout_count > 3){
//		// if so, mark this anchor as dead
//		state->anchors[state->anchor_ind].is_alive = false;
//		// clean up dead anchors
//		// decrement num_anchors by the number removed
//		int dead_anchors = remove_dead_anchors(state->anchors, state->num_anchors);
//		state->num_anchors -= dead_anchors;
//		sort_anchors_by_rx_power(state->anchors, state->num_anchors);
//		size = sprintf(debug_buf, "Removed %d dead anchors (%d anchors alive)\r\n", dead_anchors, state->num_anchors);
//		debug(&huart1, debug_buf);
//	}

	if(state->num_anchors > 0){
		enable_ranging(state); // enable ranging
	}else{
		disable_ranging(state); // no anchors, disable ranging
	}
	next_anchor(state); // increment anchor_ind, or loop back to 0
	set_state_tag_idle(state); // move to idle state
}

void anchor_wait_timeout(state_data_t* state){
	check_rx_state(); // check for and clear any errors
	size = sprintf(debug_buf, "Timed out waiting for %d\r\n", state->transact_id);
	debug(&huart1, debug_buf);
	disable_ranging(state);
	set_wait_for_poll(state);
}

int check_rx_state(){
	uint32 status_reg = dwt_read32bitreg(SYS_STATUS_ID);

	if(status_reg & SYS_STATUS_ALL_RX_ERR){
		//debug(&huart1, "RX error\r");
		if(status_reg & SYS_STATUS_RXPHE)
			debug(&huart1, "RX failed: PHY error\r\n");
		else if(status_reg & SYS_STATUS_RXFCE)
			debug(&huart1, "RX failed: FCS Error\r\n");
		else if(status_reg & SYS_STATUS_RXRFSL)
			debug(&huart1, "RX failed: Reed Solomon Frame Sync Loss\r\n");
		else if(status_reg & SYS_STATUS_RXSFDTO)
			debug(&huart1, "RX failed: SFD timeout\r\n");
		else if(status_reg & SYS_STATUS_AFFREJ)
			debug(&huart1, "RX failed: Automatic Frame Filtering rejection\r\n");
		else if(status_reg & SYS_STATUS_LDEERR)
			debug(&huart1, "RX failed: Leading edge detection processing error\r\n");

		/* Clear RX error events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
		/* Reset RX to properly reinitialise LDE operation. */
		dwt_rxreset();

		HAL_Delay(10);
		return -1;
	}
	return 0;
}

void next_anchor(state_data_t* state){
	// advance the anchor index
	state->anchor_ind++;

	if(state->anchor_ind >= state->n_range_with || state->anchor_ind >= state->num_anchors){
		state->anchor_ind = 0;
		// if we have lotsa anchors, sort them so we only range with the best
		if(state->num_anchors > state->n_range_with)
			sort_anchors_by_rx_qual(state->anchors, state->num_anchors);
	}


	size = sprintf(debug_buf, "Number of anchors    : %d\r\n", state->num_anchors);
	debug(&huart1, debug_buf);
	size = sprintf(debug_buf, "Anchors to range with: %d\r\n", state->n_range_with);
	debug(&huart1, debug_buf);
	size = sprintf(debug_buf, "Anchor index         : %d\r\n", state->anchor_ind);
	debug(&huart1, debug_buf);
}

/**
 * callback to run when the DWM_IRQn pin goes HIGH, indicating frame reception
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	//debug(&huart1, "Interrupt\r");
	if(GPIO_Pin == DW_IRQn_Pin){

		//uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);

		//if (status_reg & SYS_STATUS_RXFCG){
			state_data.new_frame = 1; // signal to the state machine that we have a new frame
			read_rx_frame(state_data.rx_buffer);
			if(state_data.rx_buffer[MAC_SIZE_EXPECTED] == ANCHOR_BROADCAST){
				process_anchor_broadcast(&state_data);
			}
		//}
	}

}

void dw_it_disable(void){
	HAL_NVIC_DisableIRQ(DW_IRQn_Type);
}
void dw_it_enable(void){
	HAL_NVIC_EnableIRQ(DW_IRQn_Type);
}


//static dwt_config_t config = {
//    2,               /* Channel number. */
//    DWT_PRF_64M,     /* Pulse repetition frequency. */
//    DWT_PLEN_128,   /* Preamble length. Used in TX only. */
//    DWT_PAC8,       /* Preamble acquisition chunk size. Used in RX only. */
//    9,               /* TX preamble code. Used in TX only. */
//    9,               /* RX preamble code. Used in RX only. */
//    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
//    DWT_BR_6M8,      /* Data rate. */
//    DWT_PHRMODE_STD, /* PHY header mode. */
//    (1025 + 64)    	 /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
//};
void init_from_config(CONFIG_FIELD_TYPE* config, dwt_config_t* dw_config){

	uint32_t id = 0;
	uint32_t channel = 2;
	uint32_t mode = 0;
	uint32_t n_anchors = 0;
	uint32_t samples_per_range = 0;
	get_field(config, FIELD_SELF_ID, (void*)&id);
	get_field(config, FIELD_MODE, (void*)&mode);
	get_field(config, FIELD_CHANNEL, (void*)&channel);
	get_field(config, FIELD_NUMBER_OF_ANCHORS, (void*)&n_anchors);
	get_field(config, FIELD_SAMPLES_PER_RANGE, (void*)&samples_per_range);
	get_field(config, FIELD_X, (void*)&state_data.x);
	get_field(config, FIELD_Y, (void*)&state_data.y);
	get_field(config, FIELD_Z, (void*)&state_data.z);

	state_data.self_id 	= (uint16_t)(id & 0xFFFF);
	state_data.channel  = (uint8_t) (channel & 0xFF);
	state_data.mode = (uint8_t) (mode & 0xFF);
	state_data.ranging_period = DFLT_RANGING_PERIOD;
	state_data.num_anchors = 0;

	if(n_anchors > MAX_NUMBER_OF_ANCHORS)
		n_anchors = MAX_NUMBER_OF_ANCHORS;

	state_data.n_range_with = (int)(n_anchors & 0xFF);

	dw_config->chan = (uint8) (channel & 0xFF);

	switch(dw_config->chan){
	case 1:
		dw_config->rxCode = 9;
		dw_config->txCode = 9;
		break;
	case 2:
		dw_config->rxCode = 9;
		dw_config->txCode = 9;
		break;
	case 3:
		dw_config->rxCode = 9;
		dw_config->txCode = 9;
		break;
	case 5:
		dw_config->rxCode = 9;
		dw_config->txCode = 9;
		break;
	case 4:
		dw_config->rxCode = 17;
		dw_config->txCode = 17;
		break;
	case 7:
		dw_config->rxCode = 17;
		dw_config->txCode = 17;
		break;
	default:
		dw_config->chan = 2;
		dw_config->rxCode = 9;
		dw_config->txCode = 9;
		break;
	}


	reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
	port_set_dw1000_slowrate();
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR){
		debug(&huart1, "INIT FAILED :(\r\n");
		return;
	}else{
		debug(&huart1, "INIT SUCCESS!!\r\n");
	}
	port_set_dw1000_fastrate();

	/* Configure DW1000. See NOTE 3 below. */
	dwt_configure(dw_config);

	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	dwt_setpanid((uint16) state_data.self_id);
	dwt_setaddress16((uint16)state_data.self_id);	// why not just have ADDRESS == PAN_ID ?

	uint16_t enable = DWT_FF_DATA_EN; //enable data frames
	if(state_data.mode == DEVICE_MODE_TAG)
		enable |= DWT_FF_BEACON_EN; // listen for these only if we're a tag

	dwt_enableframefilter(enable);

	dwt_setinterrupt(DWT_INT_RFCG, 1);

	char str[128];
	sprintf(str, "ID     : %d\r\n", state_data.self_id);
	debug(&huart1, str);
	sprintf(str, "Channel: %d\r\n", state_data.channel);
	debug(&huart1, str);
	sprintf(str, "Mode   : %d\r\n", state_data.mode);
	debug(&huart1, str);

	set_state_idle(&state_data);

}

void debug(UART_HandleTypeDef* huart, char* text){
#ifdef DEBUG
	int len = strlen(text);
	//while(text[len++] != '\n' && len < 1024){ }
//	while(HAL_UART_Transmit_IT(huart, (uint8_t*)text, len) != HAL_OK){ }
	HAL_UART_Transmit(huart, (uint8_t*)text, len, 10);
#endif
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	debug(&huart1, "Error occurred\r\n");
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
