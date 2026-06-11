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
#include "packet.h"
#include "packet_process.h"
#include "position_learning.h"
#include "transmitter.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

// Device Info
uint8_t nodeID = 4;
char nodeType = 'R';  // 'R' for Repeater, 'E' for End Device, 'G' for Gateway
double batteryPercentage = 100.0;
uint16_t distanceValue = 0; // Distance value to nearest gateway, to be updated by routing init

//Power Management Flags
volatile bool activeMode = false;
volatile bool awaitingWorAck = false;
volatile bool awaitingTransmissionEndFlag = false;
volatile bool inStandbyMode = false;


// Routing Info
NeighbourInfo_t Neighbours[MAX_NEIGHBOURS] = {0};
uint8_t NeighbourCount = 0;
uint8_t nextUptreamNodeID = 3;
uint8_t nextDownstreamNodeID = 5;
uint8_t nearestGatewayID = 1;
char direction = 'U';  // 'U' for upstream, 'D' for downstream, 'B' for broadcast

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_APP_BUFFER_SIZE          64
#define TX_TIMEOUT_VALUE             3000

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
int16_t RssiValue = 0;       // Last  Received packet Rssi
int8_t SnrValue = 0;        // Last  Received packet SNR (in Lora modulation)

// PV for I2C
static uint8_t I2CRxBuffer[MAX_APP_BUFFER_SIZE];

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
static void WakeIntMcu4Task(void);

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
  Transmitter_Init();
  PositionLearningInit();
  UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_BTN), 0, PushBtnTask);
  UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_WakeIntMcu4), 0, WakeIntMcu4Task);

  /* Arm I2C Receive */
  HAL_I2C_Slave_Receive_IT(&hi2c2, I2CRxBuffer, MAX_APP_BUFFER_SIZE);

  // if(nodeType == 'E')
  // {
  //   Transmitter_StartPeriodicED();
  // }

  // if(nodeType == 'R')
  // {
  //   CAD_Mode_Init();
  // }

  HAL_GPIO_WritePin(WAKE_MCU2_GPIO_Port, WAKE_MCU2_Pin, GPIO_PIN_RESET);

  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/*Enable ACTIVE MODE
  Enabling Active Mode including waking all other MCUs.
  Call this function upon receiving a WOR packet in CAD mode.
  */
void EnableActiveMode()
{
  activeMode = true;
  // HAL_GPIO_WritePin(WAKE_MCU2_GPIO_Port, WAKE_MCU2_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(WAKE_MCU3_GPIO_Port, WAKE_MCU3_Pin, GPIO_PIN_SET);
}

/*DISABLE ACTIVE MODE 
    if Transmit_Buffer is empty (i.e. no packet pending to be tranmitted) 
    and not awaiting a WOR Acknowlegement 
    and not awaiting a transmission end flag
    and not currently engaged in standby packet monitoring
  */ 
bool DisableActiveMode(void)
{
  if(Transmit_Buffer.count == 0U && !awaitingWorAck && !awaitingTransmissionEndFlag && !inStandbyMode)
  {
    activeMode = false;
    // HAL_GPIO_WritePin(WAKE_MCU2_GPIO_Port, WAKE_MCU2_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(WAKE_MCU3_GPIO_Port, WAKE_MCU3_Pin, GPIO_PIN_RESET);
    return true;
  }
  else
  {
    return false;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BTN_GPIO_EXTI9_Pin)
  {
    UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_BTN), CFG_SEQ_Prio_0);
  }
  else if (GPIO_Pin == WAKE_INT_MCU4_Pin)
  {
    UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_WakeIntMcu4), CFG_SEQ_Prio_0);
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2)
    {
      // Process received data here
      LoRaPacket_t receivedPacket = Packet_Decode(I2CRxBuffer);
      APP_LOG(TS_OFF, VLEVEL_M, "Received I2C packet: %s\r\n", Packet_To_String(&receivedPacket));

      // If not a broadcast packet and its nearestGwID doesn't match this repeater's nearestGwID, ignore the packet
      if (receivedPacket.direction != PACKET_DIRECTION_BROADCAST && receivedPacket.nearestGwID != nearestGatewayID)
      {
        APP_LOG(TS_OFF, VLEVEL_M, "Packet not intended for this repeater, ignoring...\r\n");
        HAL_I2C_Slave_Receive_IT(&hi2c2, I2CRxBuffer, MAX_APP_BUFFER_SIZE);
        return;
      }

      /* RECORDING RECEIVED PACKET INFO
       * If packetID not in processedPktBuf, add it to processedPktBuf
       * Else if already in processedPktBuf and this repeater's DV > packet's Tx Device's DV, add it to lowerDistanceDuplicatePktBuf
       * Else if already in processedPktBuf and this repeater's DV < packet's Tx Device's DV, add it to higherDistanceDuplicatePktBuf
      **/
      if (!PacketIDFifo_Search(&processedPktBuf, receivedPacket.packetID))
      {
        PacketIDFifo_Push(&processedPktBuf, receivedPacket.packetID);
        APP_LOG(TS_OFF, VLEVEL_M, "New packet received, added to processed buffer\r\n");
      }
      else if((distanceValue >= receivedPacket.txDistanceValue) && !PacketIDFifo_Search(&lowerDistanceDuplicatePktBuf, receivedPacket.packetID))
      {
          PacketIDFifo_Push(&lowerDistanceDuplicatePktBuf, receivedPacket.packetID);
      }
      else if((distanceValue < receivedPacket.txDistanceValue) && !PacketIDFifo_Search(&higherDistanceDuplicatePktBuf, receivedPacket.packetID))
      {
          PacketIDFifo_Push(&higherDistanceDuplicatePktBuf, receivedPacket.packetID);
      }

      HAL_I2C_Slave_Receive_IT(&hi2c2, I2CRxBuffer, MAX_APP_BUFFER_SIZE);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2)
    {
      APP_LOG(TS_OFF, VLEVEL_M, "I2C Error Callback triggered. error=0x%X\r\n", (unsigned int)HAL_I2C_GetError(&hi2c2));
      HAL_I2C_Slave_Receive_IT(&hi2c2, I2CRxBuffer, MAX_APP_BUFFER_SIZE);
    }
}

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
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

  //If in CAD mode and received a valid packet, switch to active mode
  if (!activeMode) //?? Later Update to check if it's a WOR packet with same nearestGwID
  {
    APP_LOG(TS_OFF, VLEVEL_M, "Packet detected, switching to active mode\r\n");
    EnableActiveMode();
  }

  if (receivedPacket.rxNodeID != nodeID && receivedPacket.direction != PACKET_DIRECTION_BROADCAST)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "not intended for me\r\n");
  }
  else
  {
    uint8_t nextRxID;

    if (receivedPacket.direction == PACKET_DIRECTION_DOWNSTREAM)
    {
      nextRxID = nextDownstreamNodeID;
    }
    else if (receivedPacket.direction == PACKET_DIRECTION_UPSTREAM)
    {
      nextRxID = nextUptreamNodeID;
    }
    else
    {
      nextRxID = 0; // For broadcast, set nextRxID to 0 or any value as it won't be used for routing
    }

    receivedPacket.txNodeID = nodeID;
    receivedPacket.txNodeType = (nodeType == 'E') ? PACKET_NODE_TYPE_END_DEVICE :
                                (nodeType == 'R') ? PACKET_NODE_TYPE_REPEATER :
                                                    PACKET_NODE_TYPE_GATEWAY;
    receivedPacket.txDistanceValue = distanceValue;
    receivedPacket.txBatteryPercentage = (uint8_t)batteryPercentage;
    receivedPacket.rxNodeID = nextRxID;
    receivedPacket.rxNodeType = ((receivedPacket.direction == PACKET_DIRECTION_UPSTREAM) &&
                                 (nextRxID == nearestGatewayID)) ? PACKET_NODE_TYPE_GATEWAY : PACKET_NODE_TYPE_REPEATER;
    receivedPacket.rxDistanceValue = 0;
    receivedPacket.nearestGwID = nearestGatewayID;

    if (Transmitter_Submit(&receivedPacket))
    {
      uint8_t UartMsg[512] = {0};
      packetString = Packet_To_String(&receivedPacket);
      int UartMsgSize = snprintf((char *)UartMsg, sizeof(UartMsg),
                                 "Node %d: Submitted Repeating Packet %s\r\n", nodeID, packetString);
      APP_LOG(TS_OFF, VLEVEL_M, "Submitted the received packet for repeating\r\n");
      HAL_UART_Transmit(&huart2, UartMsg, (uint16_t)UartMsgSize, HAL_MAX_DELAY);
    }
    else
    {
      APP_LOG(TS_OFF, VLEVEL_M, "repeat skipped, transmit buffer full\r\n");
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

static void PushBtnTask(void)
{
  APP_LOG(TS_OFF, VLEVEL_M, "Push Button Pressed\r\n");
  PositionLearningInitialBroadcast();
}


static void WakeIntMcu4Task(void)
{
  APP_LOG(TS_OFF, VLEVEL_M, "MCU4 Wake Int Pin Toggled\r\n");
  HAL_I2C_Slave_Receive_IT(&hi2c2, I2CRxBuffer, MAX_APP_BUFFER_SIZE);
}
/* USER CODE END PrFD */
