/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "stm32_timer.h"	//NLU$$$$$$$
#include "stm32_seq.h"		//NLU$$$$$$$
#include <string.h>			//NLU$$$$$$$
#include "utilities_def.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_APP_BUFFER_SIZE          255	//NLU$$$$$$$
#define RX_TIMEOUT_VALUE             3000	//NLU$$$$$$$
#define TX_TIMEOUT_VALUE             3000	//NLU$$$$$$$
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */
static UTIL_TIMER_Object_t TxTimer;				//NLU$$$$$$$
static uint8_t TxBuf[] = "Hello World!\r\n";	//NLU$$$$$$$
static uint8_t TxCounter = 0;					//NLU$$$$$$$
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
static void TxTimerCb(void *context);	//NLU$$$$$$$
/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */
	APP_LOG(TS_OFF, VLEVEL_M, "SubGHz App Started!\r\n");

  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */

  /* Radio Set frequency */
    Radio.SetChannel(RF_FREQUENCY);

	#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))							//NLU$$$$$$$
	  Radio.Sleep();															//NLU$$$$$$$
	  Radio.SetChannel(RF_FREQUENCY);											//NLU$$$$$$$
	  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,			//NLU$$$$$$$
						LORA_SPREADING_FACTOR, LORA_CODINGRATE,					//NLU$$$$$$$
						LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,		//NLU$$$$$$$
						true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);	//NLU$$$$$$$
	  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);				//NLU$$$$$$$
	#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))						//NLU$$$$$$$
	  Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,				//NLU$$$$$$$
						FSK_DATARATE, 0,										//NLU$$$$$$$
						FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,			//NLU$$$$$$$
						true, 0, 0, 0, TX_TIMEOUT_VALUE);						//NLU$$$$$$$
	  Radio.SetMaxPayloadLength(MODEM_FSK, MAX_APP_BUFFER_SIZE);				//NLU$$$$$$$
	#else																		//NLU$$$$$$$
	#error "Please define a modulation in the subghz_phy_app.h file."			//NLU$$$$$$$
	#endif /* USE_MODEM_LORA | USE_MODEM_FSK */									//NLU$$$$$$$

  /* Periodic TX every 2000 ms */
    UTIL_TIMER_Create(&TxTimer, 2000, UTIL_TIMER_PERIODIC, TxTimerCb, NULL);	//NLU$$$$$$$
    UTIL_TIMER_Start(&TxTimer);													//NLU$$$$$$$

  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
	APP_LOG(TS_OFF, VLEVEL_M, "TX done\r\n");	//NLU$$$$$$$
  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
	APP_LOG(TS_OFF, VLEVEL_M, "RX done\r\n");	//NLU$$$$$$$
  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
	APP_LOG(TS_OFF, VLEVEL_M, "TX timeout\r\n");	//NLU$$$$$$$
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
	APP_LOG(TS_OFF, VLEVEL_M, "RX timeout\r\n");	//NLU$$$$$$$
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
	APP_LOG(TS_OFF, VLEVEL_M, "RX Error\r\n");	//NLU$$$$$$$
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */
static void TxTimerCb(void *context)					//NLU$$$$$$$
{														//NLU$$$$$$$
  (void)context;										//NLU$$$$$$$
  HAL_GPIO_TogglePin(LED1_1_GPIO_Port, LED1_1_Pin);		//NLU$$$$$$$
  TxCounter = TxCounter +1;								//NLU$$$$$$$
  APP_LOG(TS_OFF, VLEVEL_M, "TX counter = \r\n");	//NLU$$$$$$$
  Radio.Send(TxBuf, 64);		//NLU$$$$$$$
//  UTIL_TIMER_Start(&TxTimer);							//NLU$$$$$$$
}														//NLU$$$$$$$

/* USER CODE END PrFD */
