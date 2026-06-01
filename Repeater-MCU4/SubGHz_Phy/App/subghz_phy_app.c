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
#include "i2c.h"

#include "cad_mode.h"
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
#define MAX_APP_BUFFER_SIZE          255
#define MCU1_I2C_ADDRESS_7BIT        1
#define WAKE_MCU1_WAKEUP_DELAY_MS    10
#define WAKE_MCU1_RELEASE_DELAY_MS   2000
#define I2C_BUSY_RETRY_TIMEOUT_MS    2000
#define I2C_BUSY_RETRY_MAX_DELAY_MS  20
#define I2C_TX_TIMEOUT_MS            100
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

// PV for I2C communication with MCU1
static UTIL_TIMER_Object_t WakeMcu1ReleaseTimer;

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
static HAL_StatusTypeDef WakeMCU1andTransferData(uint8_t *data, uint16_t size);
static void WakeMcu1ReleaseTimerCb(void *context);

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
  RadioEvents.CadDone = CAD_Mode_OnCadDone;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */

  /* Radio Configuration for LoRa */
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                    true, 0, 0, LORA_IQ_INVERSION_ON, true);
  CAD_Mode_ConfigRadio();
  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
  Radio.Sleep();

  /*  Register Sequencer Tasks */
  UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_BTN), 0, PushBtnTask);

  UTIL_TIMER_Create(&WakeMcu1ReleaseTimer, WAKE_MCU1_RELEASE_DELAY_MS,
                    UTIL_TIMER_ONESHOT, WakeMcu1ReleaseTimerCb, NULL);

  /* Initiate CAD Mode */
  CAD_Mode_Init();

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
    return;
  }

  memcpy(RxTextBuf, payload, RxBufferSize);

  if (RxBufferSize < LORA_PACKET_HEADER_SIZE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "RX packet too short, size=%u\r\n", RxBufferSize);
    return;
  }

  receivedPacket = Packet_Decode(RxTextBuf);
  packetString = Packet_To_String(&receivedPacket);
  APP_LOG(TS_OFF, VLEVEL_M, "RX done, size=%u, RSSI=%d, SNR=%d, %s\r\n",
          size, rssi, LoraSnr_FskCfo, packetString);

  Radio.Sleep();
  
  WakeMCU1andTransferData(payload, size);
  
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
  Radio.Sleep();
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
  APP_LOG(TS_OFF, VLEVEL_M, "RX Error\r\n");
  Radio.Sleep();
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */

static HAL_StatusTypeDef WakeMCU1andTransferData(uint8_t *data, uint16_t size)
{
  HAL_StatusTypeDef status;
  uint32_t retryStartTick;
  uint32_t retryDelay;

  // Wake up MCU1
  HAL_GPIO_WritePin(WAKE_MCU1_GPIO_Port, WAKE_MCU1_Pin, GPIO_PIN_SET);
  HAL_Delay(WAKE_MCU1_WAKEUP_DELAY_MS);

  //If I2C bus is busy, wait until it's free
  retryStartTick = HAL_GetTick();
  while (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY) != RESET)
  {
    if ((HAL_GetTick() - retryStartTick) >= I2C_BUSY_RETRY_TIMEOUT_MS)
    {
      APP_LOG(TS_OFF, VLEVEL_M, "I2C2 busy timeout before MCU1 transfer\r\n");
      HAL_GPIO_WritePin(WAKE_MCU1_GPIO_Port, WAKE_MCU1_Pin, GPIO_PIN_RESET);
      return HAL_TIMEOUT;
    }
    retryDelay = ((uint32_t)rand() % I2C_BUSY_RETRY_MAX_DELAY_MS) + 1U;
    HAL_Delay(retryDelay);
  }

  // Transfer data to MCU1 via I2C
  status = HAL_I2C_Master_Transmit(&hi2c2,
                                   (uint16_t)(MCU1_I2C_ADDRESS_7BIT << 1),
                                   data,
                                   size,
                                   I2C_TX_TIMEOUT_MS);

  // If transfer is successful, start timer to release MCU1 from wakeup after a delay
  if (status == HAL_OK)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "Data transferred to MCU1\r\n");
    (void)UTIL_TIMER_Start(&WakeMcu1ReleaseTimer);
  }
  else
  {
    HAL_GPIO_WritePin(WAKE_MCU1_GPIO_Port, WAKE_MCU1_Pin, GPIO_PIN_RESET);
    APP_LOG(TS_OFF, VLEVEL_M, "Failed to transfer data to MCU1\r\n");
  }

  return status;
}


static void WakeMcu1ReleaseTimerCb(void *context)
{
  (void)context;
  HAL_GPIO_WritePin(WAKE_MCU1_GPIO_Port, WAKE_MCU1_Pin, GPIO_PIN_RESET);
  APP_LOG(TS_OFF, VLEVEL_M, "MCU1 released from wakeup\r\n");
}


static void PushBtnTask(void)
{
  APP_LOG(TS_OFF, VLEVEL_M, "Push Button Pressed\r\n");
}

/* USER CODE END PrFD */
