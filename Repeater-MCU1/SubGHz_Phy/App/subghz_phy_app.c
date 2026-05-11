/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  Nalith Udugampola
  * @brief   Application of the LoRa Repeater's SubGHz_Phy Middleware
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
#include "usart.h"
#include "stm32_timer.h"
#include "stm32_seq.h"
#include <string.h>
#include "radio_driver.h"
#include "utilities_def.h"
#include <stdio.h>

#include "cad_mode.h"
#include "transmitter.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

// Device Info
uint8_t nodeID = 3;
char nodeType = 'E';  // 'R' for Repeater, 'E' for End Device, 'G' for Gateway
double batteryPercentage = 100.0;
uint16_t distanceValue = 0; // Distance value to nearest gateway, to be updated by routing init

// Routing Info
NeighbourInfo_t Neighbours[MAX_NEIGHBOURS] = {0};
uint8_t NeighbourCount = 0;
uint8_t nextUptreamNodeID = 4;
uint8_t nextDownstreamNodeID = 2;
uint8_t nearestGatewayID = 0;
char direction = 'U';  // 'U' for upstream, 'D' for downstream, 'B' for broadcast

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_APP_BUFFER_SIZE          255
#define TX_TIMEOUT_VALUE             3000

#define MCU2_I2C_ADDRESS_7BIT        12
#define I2C_TX_TIMEOUT_MS            100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */

// PV for Rx data
static uint8_t RxTextBuf[MAX_APP_BUFFER_SIZE];
uint16_t RxBufferSize = 0;  // Last  Received Buffer Size
int8_t RssiValue = 0;       // Last  Received packer Rssi
int8_t SnrValue = 0;        // Last  Received packer SNR (in Lora modulation)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

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
static HAL_StatusTypeDef WakeMcu2AndSendString(const char *message);

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */
  APP_LOG(TS_OFF, VLEVEL_M, "SubGHz App Started!\r\n");

  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = Transmitter_OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = Transmitter_OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  RadioEvents.CadDone = CAD_Mode_OnCadDone;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */

  /* Radio Configuration for LoRa */
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
                    true, 0, 0, LORA_IQ_INVERSION_ON, false);
  CAD_Mode_ConfigRadio();
  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
  Radio.Sleep();

  /*  Register Sequencer Tasks */
  UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_BTN), 0, PushBtnTask);

  if(nodeType == 'E')
  {
    Transmitter_StartPeriodicED();
  }

  if(nodeType == 'R')
  {
    CAD_Mode_Init();
  }

  HAL_GPIO_WritePin(WAKE_MCU2_GPIO_Port, WAKE_MCU2_Pin, GPIO_PIN_RESET);

  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BTN_GPIO_EXTI9_Pin)
  {
    UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_BTN), CFG_SEQ_Prio_0);
  }
}

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */

  APP_LOG(TS_OFF, VLEVEL_M, "RX done\r\n");

  /* Clear BufferRx*/
  memset(RxTextBuf, 0, MAX_APP_BUFFER_SIZE);

  /* Record payload size*/
  RxBufferSize = size;
  if (RxBufferSize <= MAX_APP_BUFFER_SIZE)
  {
    memcpy(RxTextBuf, payload, RxBufferSize);
  }

  uint8_t txID = 0;
  uint8_t intendedRxID = 0;
  char Direction = 0;
  char Payload[MAX_APP_BUFFER_SIZE] = {0};

  char *txIDStart = (char *)RxTextBuf;
  if (*txIDStart == '|')
  {
    char *txIDEnd = strchr(txIDStart + 1, '|');
    char *intendedRxIDEnd = (txIDEnd != NULL) ? strchr(txIDEnd + 1, '|') : NULL;
    char *directionEnd = (intendedRxIDEnd != NULL) ? strchr(intendedRxIDEnd + 1, '|') : NULL;

    if ((txIDEnd != NULL) && (intendedRxIDEnd != NULL) && (directionEnd != NULL))
    {
      for (char *p = txIDStart + 1; p < txIDEnd; p++)
      {
        txID = (uint8_t)((txID * 10U) + (uint8_t)(*p - '0'));
      }

      for (char *p = txIDEnd + 1; p < intendedRxIDEnd; p++)
      {
        intendedRxID = (uint8_t)((intendedRxID * 10U) + (uint8_t)(*p - '0'));
      }

      Direction = *(intendedRxIDEnd + 1);

      size_t payloadLen = strlen(directionEnd + 1);
      if ((payloadLen >= 2U) &&
          (directionEnd[1 + payloadLen - 2U] == '\r') &&
          (directionEnd[1 + payloadLen - 1U] == '\n'))
      {
        payloadLen -= 2U;
      }

      if (payloadLen >= MAX_APP_BUFFER_SIZE)
      {
        payloadLen = MAX_APP_BUFFER_SIZE - 1U;
      }

      memcpy(Payload, directionEnd + 1, payloadLen);
    }
  }

  APP_LOG(TS_OFF, VLEVEL_M, "RX done, size=%u, RSSI=%d, SNR=%d, Tx ID=%u, Intended Rx ID=%u, Direction=%c, Payload=\"%s\"\r\n",
          size, rssi, LoraSnr_FskCfo, txID, intendedRxID, Direction, Payload);

  if (intendedRxID != nodeID)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "not intended for me\r\n");
  }
  else
  {
    uint8_t nextRxID = (Direction == 'U') ? nextUptreamNodeID : nextDownstreamNodeID;
    uint8_t RepeatBuf[MAX_APP_BUFFER_SIZE] = {0};
    int RepeatSize = snprintf((char *)RepeatBuf, sizeof(RepeatBuf), "|%d|%d|%c|%s\r\n",
                              nodeID, nextRxID, Direction, Payload);

    if ((RepeatSize > 0) && (RepeatSize < MAX_APP_BUFFER_SIZE))
    {
      APP_LOG(TS_OFF, VLEVEL_M, "Repeating the received packet\r\n");

      uint8_t UartMsg[MAX_APP_BUFFER_SIZE] = {0};
      int UartMsgSize = snprintf((char *)UartMsg, sizeof(UartMsg),"Node %d: Repeating Packet %s\r\n", nodeID, RepeatBuf);
      HAL_UART_Transmit(&huart2, UartMsg, (uint16_t)UartMsgSize, HAL_MAX_DELAY);
      
      Radio.Send(RepeatBuf, (uint16_t)RepeatSize);
    }
  }
  
  /* USER CODE END OnRxDone */
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

static HAL_StatusTypeDef WakeMcu2AndSendString(const char *message)
{
  HAL_StatusTypeDef status;
  uint16_t messageLength;

  if (message == NULL)
  {
    return HAL_ERROR;
  }

  messageLength = (uint16_t)strlen(message);
  if (messageLength == 0U)
  {
    return HAL_OK;
  }

  HAL_GPIO_WritePin(WAKE_MCU2_GPIO_Port, WAKE_MCU2_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(WAKE_MCU2_GPIO_Port, WAKE_MCU2_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);

  status = HAL_I2C_Master_Transmit(&hi2c2,
                                   (uint16_t)(MCU2_I2C_ADDRESS_7BIT << 1),
                                   (uint8_t *)message,
                                   messageLength,
                                   I2C_TX_TIMEOUT_MS);

  return status;
}

static void PushBtnTask(void)
{
  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

  if (WakeMcu2AndSendString("Hello World from MCU1") == HAL_OK)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "I2C message sent to MCU2\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "I2C message send to MCU2 failed\r\n");
  }
}
/* USER CODE END PrFD */
