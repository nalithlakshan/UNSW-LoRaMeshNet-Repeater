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
#include "i2c.h"
#include "main.h"
#include "stm32_timer.h"
#include "stm32_seq.h"
#include <string.h>
#include "radio_driver.h"
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
#define MAX_APP_BUFFER_SIZE          255
#define RX_TIMEOUT_VALUE             3000
#define TX_TIMEOUT_VALUE             3000
#define CAD_SCAN_PERIOD_MS           1000
#define CAD_DET_PEAK                 28
#define CAD_DET_MIN                  14
#define MCU1_I2C_RX_TIMEOUT_MS       1000
#define MCU1_EXPECTED_MSG_LEN        (sizeof("Hello World from MCU1") - 1U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */
// PV for periodic transmission example
static UTIL_TIMER_Object_t TxTimer;
static uint8_t TxBuf[] = "Hello World!\r\n";
static uint8_t TxCounter = 0;

// PV for periodic CAD scan example
static UTIL_TIMER_Object_t CadTimer;
static uint32_t CadScanCounter = 0;

// PV for Rx data
static uint8_t RxTextBuf[MAX_APP_BUFFER_SIZE];
uint16_t RxBufferSize = 0;  // Last  Received Buffer Size
int8_t RssiValue = 0;       // Last  Received packer Rssi
int8_t SnrValue = 0;        // Last  Received packer SNR (in Lora modulation)
static uint8_t Mcu1I2cRxBuf[MCU1_EXPECTED_MSG_LEN + 1U];
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

/**
  * @brief Function executed on Radio CAD Done event
  * @param  channelActivityDetected true when activity is detected on the channel
  */
static void OnCadDone(bool channelActivityDetected);

static void TxTimerCb(void *context);
static void TxTask(void);

static void CadTimerCb(void *context);
static void CAD_Scan(void);
static void PushBtnTask(void);
static void Mcu1WakeRxTask(void);
static void ReceiveI2cMessageFromMcu1(void);

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

#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                    true, 0, 0, LORA_IQ_INVERSION_ON, false);
  SUBGRF_SetCadParams(LORA_CAD_02_SYMBOL, CAD_DET_PEAK, CAD_DET_MIN,
                      LORA_CAD_RX, 0);
  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
  Radio.Sleep();
#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
  Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                    FSK_DATARATE, 0,
                    FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, 0, TX_TIMEOUT_VALUE);
  Radio.SetMaxPayloadLength(MODEM_FSK, MAX_APP_BUFFER_SIZE);
#else
#error "Please define a modulation in the subghz_phy_app.h file."
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */

  UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_LoRaTx), 0, TxTask);
  UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_LoRaCadScan), 0, CAD_Scan);
  UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_BTN), 0, PushBtnTask);
  UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_MCU1WakeRx), 0, Mcu1WakeRxTask);

  /* Periodic TX every 2000 ms */
  UTIL_TIMER_Create(&TxTimer, 5000, UTIL_TIMER_PERIODIC, TxTimerCb, NULL);
  // UTIL_TIMER_Start(&TxTimer);

  /* Periodic LoRa CAD scan every 1000 ms */
  UTIL_TIMER_Create(&CadTimer, CAD_SCAN_PERIOD_MS, UTIL_TIMER_PERIODIC, CadTimerCb, NULL);
  UTIL_TIMER_Start(&CadTimer);

  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BTN_GPIO_EXTI9_Pin)
  {
    UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_BTN), CFG_SEQ_Prio_0);
  }
  else if (GPIO_Pin == WAKE_INT_MCU1_Pin)
  {
    UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_MCU1WakeRx), CFG_SEQ_Prio_0);
  }
}

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
  APP_LOG(TS_OFF, VLEVEL_M, "TX done\r\n");
  Radio.SetChannel(RF_FREQUENCY);
  Radio.Sleep();
  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */

  /* Clear BufferRx*/
  memset(RxTextBuf, 0, MAX_APP_BUFFER_SIZE);

  /* Record payload size*/
  RxBufferSize = size;
  if (RxBufferSize <= MAX_APP_BUFFER_SIZE)
  {
    memcpy(RxTextBuf, payload, RxBufferSize);
  }

  APP_LOG(TS_OFF, VLEVEL_M, "RX done, size=%u, RSSI=%d, SNR=%d, payload=\"%s\"\r\n",
          size, rssi, LoraSnr_FskCfo, RxTextBuf);

  // Repeating the received packet after receiving it
  APP_LOG(TS_OFF, VLEVEL_M, "Repeating the received packet\r\n");
  Radio.SetChannel(RF_FREQUENCY+250000);
  Radio.Send(payload, size);
  
  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
  APP_LOG(TS_OFF, VLEVEL_M, "TX timeout\r\n");
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

static void TxTimerCb(void *context)
{
  (void)context;
  APP_LOG(TS_OFF, VLEVEL_M, "TxTimer Expired\r\n");
  UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_LoRaTx), CFG_SEQ_Prio_0);
}

static void TxTask(void)
{
//  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  if (Radio.GetStatus() != RF_IDLE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "TX skipped, radio busy\r\n");
    return;
  }

  TxCounter = TxCounter + 1U;
  APP_LOG(TS_OFF, VLEVEL_M, "TX counter = %u\r\n", TxCounter);
  Radio.Send(TxBuf, PAYLOAD_LEN);
  // int i = 0;
  // while(i<10000000){
	//   i = i+1;
  // }
}

static void OnCadDone(bool channelActivityDetected)
{
  /* USER CODE BEGIN OnCadDone */
  APP_LOG(TS_OFF, VLEVEL_M, "CAD done: %s\r\n",
          channelActivityDetected ? "activity detected" : "channel clear");

  if (channelActivityDetected == false)
  {
    Radio.Sleep();
  }
  else
  {
    // Channel activity detected, we can decide to receive or transmit
    // For example, we can start receiving
    Radio.Rx(RX_TIMEOUT_VALUE);
  }
  /* USER CODE END OnCadDone */
}

static void CadTimerCb(void *context)
{
  (void)context;
  UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_LoRaCadScan), CFG_SEQ_Prio_0);
}

static void CAD_Scan(void)
{
  if (Radio.GetStatus() != RF_IDLE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "CAD skipped, radio busy\r\n");
    return;
  }

  CadScanCounter++;
  APP_LOG(TS_OFF, VLEVEL_M, "CAD scan #%u\r\n", (unsigned int)CadScanCounter);
  Radio.StartCad();
}

static void ReceiveI2cMessageFromMcu1(void)
{
  HAL_StatusTypeDef status;

  memset(Mcu1I2cRxBuf, 0, sizeof(Mcu1I2cRxBuf));
  status = HAL_I2C_Slave_Receive(&hi2c2,
                                 Mcu1I2cRxBuf,
                                 MCU1_EXPECTED_MSG_LEN,
                                 MCU1_I2C_RX_TIMEOUT_MS);

  if (status == HAL_OK)
  {
    Mcu1I2cRxBuf[MCU1_EXPECTED_MSG_LEN] = '\0';
    APP_LOG(TS_OFF, VLEVEL_M, "I2C message from MCU1: \"%s\"\r\n", Mcu1I2cRxBuf);
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "I2C receive from MCU1 failed, status=%d, error=0x%X\r\n",
            (unsigned int)status, (unsigned int)HAL_I2C_GetError(&hi2c2));
  }
}

static void Mcu1WakeRxTask(void)
{
  APP_LOG(TS_OFF, VLEVEL_M, "Wake interrupt from MCU1 detected\r\n");
  ReceiveI2cMessageFromMcu1();
}

static void PushBtnTask(void)
{
  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}
/* USER CODE END PrFD */
