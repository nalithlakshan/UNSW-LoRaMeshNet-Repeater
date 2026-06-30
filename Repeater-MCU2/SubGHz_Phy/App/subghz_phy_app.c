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
#include "main.h"
#include "stm32_timer.h"
#include "stm32_seq.h"
#include <string.h>
#include <stdlib.h>
#include "radio_driver.h"
#include "utilities_def.h"
#include "usart.h"

#include "i2c_pkt_transfer.h"
#include "packet.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */

// PV for Rx WOR packets
static uint8_t RxTextBuf[MAX_APP_BUFFER_SIZE];
uint16_t RxBufferSize = 0;  // Last  Received Buffer Size
int8_t RssiValue = 0;       // Last  Received packer Rssi
int8_t SnrValue = 0;        // Last  Received packer SNR (in Lora modulation)

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

static void PushBtnTask(void);
static void WakeIntMcu1TTask(void);

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

  /* Radio Configuration for LoRa */
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                    true, 0, 0, LORA_IQ_INVERSION_ON, true);
  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
  Radio.Sleep();

  /*  Register Sequencer Tasks */
  UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_BTN), 0, PushBtnTask);
  UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_WakeIntMcu1), 0, WakeIntMcu1TTask);
  I2cPktTransfer_Init();

  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

//Push button interrupt handling
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BTN_GPIO_EXTI9_Pin)
  {
    UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_BTN), CFG_SEQ_Prio_0);
  }
  else if (GPIO_Pin == WAKE_INT_MCU1_Pin)
  {
    UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_WakeIntMcu1), CFG_SEQ_Prio_0);
  }
}

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */

  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
  LoRaPacket_t receivedPacket;
  const char *packetString;

  /* Clear BufferRx*/
  memset(RxTextBuf, 0, MAX_APP_BUFFER_SIZE);

  /* Record payload size*/
  RxBufferSize = size;
  if (RxBufferSize > MAX_APP_BUFFER_SIZE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "RX packet too large, size=%u\r\n", RxBufferSize);
    Radio.Rx(0);
    return;
  }

  memcpy(RxTextBuf, payload, RxBufferSize);

  if (RxBufferSize < LORA_PACKET_HEADER_SIZE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "RX packet too short, size=%u\r\n", RxBufferSize);
    Radio.Rx(0);
    return;
  }

  receivedPacket = Packet_Decode(RxTextBuf);
  packetString = Packet_To_String(&receivedPacket);
  APP_LOG(TS_OFF, VLEVEL_M, "RX done, size=%u, RSSI=%d, SNR=%d, %s\r\n",
          size, rssi, LoraSnr_FskCfo, packetString);

  Radio.Rx(0);
  
  if (!I2cPktTransfer_Enqueue(RxTextBuf))
  {
    APP_LOG(TS_OFF, VLEVEL_M, "I2C packet FIFO full, packet dropped\r\n");
  }
  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
  
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
  APP_LOG(TS_OFF, VLEVEL_M, "RX timeout\r\n");
  Radio.Rx(0);
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
  APP_LOG(TS_OFF, VLEVEL_M, "RX Error\r\n");
  Radio.Rx(0);
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */

static void PushBtnTask(void)
{
  APP_LOG(TS_OFF, VLEVEL_M, "Push Button Pressed\r\n");
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                    true, 0, 0, LORA_IQ_INVERSION_ON, true);
  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
  Radio.Standby();
  HAL_Delay(100);
  Radio.Rx(0); // Go to Rx mode to receive on DATA-RP channel
}

static void WakeIntMcu1TTask(void)
{
  APP_LOG(TS_OFF, VLEVEL_M, "\nWake interrupt from MCU1\r\n");
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                    true, 0, 0, LORA_IQ_INVERSION_ON, true);
  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
  Radio.Standby();
  HAL_Delay(100);
  Radio.Rx(0); // Go to Rx mode to receive on DATA-RP channel
}

/* USER CODE END PrFD */
