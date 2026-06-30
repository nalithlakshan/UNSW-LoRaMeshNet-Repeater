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
#include "idle_timer.h"
#include "packet.h"
#include "packet_process.h"
#include "position_learning.h"
#include "transmitter.h"
#include "wor_ack_wait.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

// Device Info
uint8_t nodeID = 4;
char nodeType = 'R';  // 'R' for Repeater, 'E' for End Device, 'G' for Gateway
double batteryPercentage = 100.0;
uint16_t distanceValue = 0; // Distance value to nearest gateway, to be updated by routing init
uint8_t sequenceNumber = 0;

// Power Management Settings
const uint32_t MCU1_IDLE_SLEEP_DELAY_MS = 5000U;


// Routing Info
NeighbourInfo_t Neighbours[MAX_NEIGHBOURS] = {0};
uint8_t NeighbourCount = 0;
uint8_t nextUptreamNodeID = 0;
uint8_t nextDownstreamNodeID = 0;
uint8_t nearestGatewayID = 0;
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
  // Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
  //                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
  //                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0,
  //                   true, 0, 0, LORA_IQ_INVERSION_ON, false);
  CAD_Mode_ConfigRadio();
  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
  Radio.Sleep();

  /*  Register Sequencer Tasks */
  Transmitter_Init();
  PositionLearningInit();
  PacketProcess_Init();
  WorAckWait_Init();
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

  HAL_GPIO_WritePin(WAKE_MCU2_GPIO_Port, WAKE_MCU2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(WAKE_MCU3_GPIO_Port, WAKE_MCU3_Pin, GPIO_PIN_SET);

  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

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

void SubghzApp_RearmI2cRx(void)
{
  HAL_I2C_Slave_Receive_IT(&hi2c2, I2CRxBuffer, MAX_APP_BUFFER_SIZE);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2)
    {
      // Process received data here
      LoRaPacket_t receivedPacket = Packet_Decode(I2CRxBuffer);
      APP_LOG(TS_OFF, VLEVEL_M, "Received I2C packet: %s\r\n", Packet_To_String(&receivedPacket));

      // Ignore packets sent by this repeater's own trasnmitter
      if (receivedPacket.txNodeID == nodeID){
        APP_LOG(TS_OFF, VLEVEL_M, "It is from own Tx. Therefore, Igored!\r\n");
        HAL_I2C_Slave_Receive_IT(&hi2c2, I2CRxBuffer, MAX_APP_BUFFER_SIZE);
        return;
      }

      //handler of awaiting WOR-ACK before forwarding DATA packets
      WorAckWait_HandleReceivedPacket(&receivedPacket);

      // If waiting to send a WOR-ACK to an ED and another repeater already did that. 
      if ((receivedPacket.packetType == PACKET_TYPE_WOR) &&(receivedPacket.ackNodeID != 0U) &&
          PacketIDFifo_Remove(&pendingWorAckNodes, receivedPacket.ackNodeID))
      {
        APP_LOG(TS_OFF, VLEVEL_M, "Removed pending WOR ACK node %u\r\n", receivedPacket.ackNodeID);
      }

      // If not a broadcast packet and not from an ED and its nearestGwID doesn't match this repeater's nearestGwID, ignore the packet
      if ((receivedPacket.txNodeType != PACKET_NODE_TYPE_END_DEVICE) &&
          (receivedPacket.direction != PACKET_DIRECTION_BROADCAST) &&
          (receivedPacket.nearestGwID != nearestGatewayID))
      {
        APP_LOG(TS_OFF, VLEVEL_M, "Packet (nearest GW: %u) not intended for this device (nearest GW: %u)\r\n", receivedPacket.nearestGwID, nearestGatewayID);
        HAL_I2C_Slave_Receive_IT(&hi2c2, I2CRxBuffer, MAX_APP_BUFFER_SIZE);
        return;
      }

#if (REJECT_UPSTREAM_ALREADY_FORWARDED_PKTS == 1U)
      if ((receivedPacket.direction == PACKET_DIRECTION_UPSTREAM) &&
          (receivedPacket.rxNodeID != 0U) &&
          (receivedPacket.rxDistanceValue < distanceValue) &&
          !PacketIDFifo_Search(&upstreamAlreadyForwardedPktBuf, receivedPacket.packetID))
      {
        PacketIDFifo_Push(&upstreamAlreadyForwardedPktBuf, receivedPacket.packetID);
        APP_LOG(TS_OFF, VLEVEL_M, "Upstream packet already heard forwarded ahead: %u/%u\r\n",
                (uint8_t)(receivedPacket.packetID >> 8),
                (uint8_t)receivedPacket.packetID);
      }
#endif

      /* RECORDING RECEIVED PACKET INFO
       * If packetID not in processedPktBuf, add it to processedPktBuf
       * Else if already in processedPktBuf and this repeater's DV > packet's Tx Device's DV, add it to lowerDistanceDuplicatePktBuf
       * Else if already in processedPktBuf and this repeater's DV < packet's Tx Device's DV, add it to higherDistanceDuplicatePktBuf
      **/
      if (!PacketIDFifo_Search(&processedPktBuf, receivedPacket.packetID))
      {
        if(!(((receivedPacket.direction == PACKET_DIRECTION_UPSTREAM) && 
            (receivedPacket.rxNodeID != nodeID) && (receivedPacket.rxNodeID != 0U) &&
            (receivedPacket.rxDistanceValue > distanceValue))
            ||
            ((receivedPacket.direction == PACKET_DIRECTION_DOWNSTREAM) && 
            (receivedPacket.rxNodeID != nodeID) && (receivedPacket.rxNodeID != 0U) &&
            (receivedPacket.rxDistanceValue < distanceValue))))
            {
              PacketIDFifo_Push(&processedPktBuf, receivedPacket.packetID);
            }
        // PacketIDFifo_Push(&processedPktBuf, receivedPacket.packetID);
        PacketFifo_Push(&rxBuffer, &receivedPacket);
        PacketProcess_Schedule();
        APP_LOG(TS_OFF, VLEVEL_M, "New packet received!\r\n");
      }
      else
      { 
        //Exception for Position Learning Phase 1 Packets 
        if ((receivedPacket.positionLearningMode == 1U) && (receivedPacket.payloadSize > 0U) && 
            (receivedPacket.payload[0] == 1U))
        {
          PacketFifo_Push(&rxBuffer, &receivedPacket);
          PacketProcess_Schedule();
          APP_LOG(TS_OFF, VLEVEL_M, "Duplicate PL1 packet submitted for processing\r\n");
        }

        if ((distanceValue >= receivedPacket.txDistanceValue) &&
            !PacketIDFifo_Search(&lowerDistanceDuplicatePktBuf, receivedPacket.packetID))
        {
          PacketIDFifo_Push(&lowerDistanceDuplicatePktBuf, receivedPacket.packetID);
          APP_LOG(TS_OFF, VLEVEL_M, "Lower distance duplicate packet received \r\n");
        }
        else if ((distanceValue < receivedPacket.txDistanceValue) &&
                 !PacketIDFifo_Search(&higherDistanceDuplicatePktBuf, receivedPacket.packetID))
        {
          PacketIDFifo_Push(&higherDistanceDuplicatePktBuf, receivedPacket.packetID);
          APP_LOG(TS_OFF, VLEVEL_M, "Higher distance duplicate packet received \r\n");
        }
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
  APP_LOG(TS_OFF, VLEVEL_M, "RX Done\r\n");
  Radio.Sleep();
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
  PositionLearningReset();
  PositionLearningInitialBroadcast();
}


static void WakeIntMcu4Task(void)
{
  IdleTimer_RestartIfRunning();

  APP_LOG(TS_OFF, VLEVEL_M, "\nMCU4 Wake Int Pin Toggled\r\n");
  MX_I2C2_Init();
  SubghzApp_RearmI2cRx();
}
/* USER CODE END PrFD */
