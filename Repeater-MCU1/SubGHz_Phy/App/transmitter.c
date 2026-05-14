/*
 * transmitter.c
 *
 *  Created on: May 10, 2026
 *      Author: Nalith
 */

#include "transmitter.h"

#include "packet.h"
#include "radio.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "subghz_phy_app.h"
#include "sys_app.h"
#include "usart.h"
#include "utilities_def.h"

#include <stdio.h>
#include <string.h>

#define TRANSMITTER_PERIOD_MS        8000
#define TRANSMITTER_BUFFER_SIZE      256
#define TRANSMITTER_UART_BUFFER_SIZE 512
#define DEBUG_TX                     1

static UTIL_TIMER_Object_t TxTimer;
static LoRaPacket_t TxPacket;
static uint8_t TxBuf[TRANSMITTER_BUFFER_SIZE];
static uint16_t TxBufSize = 0;
static uint8_t TxCounter = 0;

static void TxTimerCb(void *context);
static void TxTask(void);

void Transmitter_StartPeriodicED(void)
{
    const char payload[] = "Hello World!";

    UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_LoRaTx), 0, TxTask);

    TxPacket.txNodeID = nodeID;
    TxPacket.txNodeType = PACKET_NODE_TYPE_END_DEVICE;
    TxPacket.txDistanceValue = distanceValue;
    TxPacket.txBatteryPercentage = (uint8_t)batteryPercentage;
    TxPacket.rxNodeID = nextUptreamNodeID;
    TxPacket.rxNodeType = PACKET_NODE_TYPE_REPEATER;
    TxPacket.rxDistanceValue = 0;
    TxPacket.ackNodeID = 0;
    TxPacket.nearestGwID = nearestGatewayID;
    TxPacket.packetType = PACKET_TYPE_DATA;
    TxPacket.direction = PACKET_DIRECTION_UPSTREAM;
    TxPacket.preambleSize = LORA_PREAMBLE_LENGTH;
    TxPacket.payloadSize = (uint16_t)strlen(payload);
    memcpy(TxPacket.payload, payload, TxPacket.payloadSize);

    TxBufSize = Packet_Encode(&TxPacket, TxBuf, sizeof(TxBuf));

    UTIL_TIMER_Create(&TxTimer, TRANSMITTER_PERIOD_MS, UTIL_TIMER_PERIODIC, TxTimerCb, NULL);
    UTIL_TIMER_Start(&TxTimer);
}

void Transmitter_OnTxDone(void)
{
    if (DEBUG_TX)
    {
        APP_LOG(TS_OFF, VLEVEL_M, "TX done\r\n");
    }
    Radio.SetChannel(RF_FREQUENCY);
    Radio.Sleep();
}

void Transmitter_OnTxTimeout(void)
{
    if (DEBUG_TX)
    {
        APP_LOG(TS_OFF, VLEVEL_M, "TX timeout\r\n");
    }
}



static void TxTimerCb(void *context)
{
    (void)context;
    if (DEBUG_TX)
    {
        APP_LOG(TS_OFF, VLEVEL_M, "TxTimer Expired\r\n");
    }
    UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_LoRaTx), CFG_SEQ_Prio_0);
}

static void TxTask(void)
{
    if (Radio.GetStatus() != RF_IDLE)
    {
        if (DEBUG_TX)
        {
            APP_LOG(TS_OFF, VLEVEL_M, "TX skipped, radio busy\r\n");
        }
        return;
    }

    TxCounter = TxCounter + 1U;
    if (DEBUG_TX)
    {
        APP_LOG(TS_OFF, VLEVEL_M, "TX counter = %u\r\n", TxCounter);
    }
    uint8_t UartMsg[TRANSMITTER_UART_BUFFER_SIZE] = {0};
    LoRaPacket_t decodedPacket;
    const char *packetString;

    TxPacket.packetID = (uint16_t)(((uint16_t)nodeID << 8) | TxCounter);
    TxBufSize = Packet_Encode(&TxPacket, TxBuf, sizeof(TxBuf));
    if (TxBufSize == 0U)
    {
        if (DEBUG_TX)
        {
            APP_LOG(TS_OFF, VLEVEL_M, "TX skipped, packet encode failed\r\n");
        }
        return;
    }

    decodedPacket = Packet_Decode(TxBuf);
    packetString = Packet_To_String(&decodedPacket);
    int UartMsgSize = snprintf((char *)UartMsg, sizeof(UartMsg),
                                "Node %d: Transmitting Packet %s\r\n", nodeID, packetString);
    HAL_UART_Transmit(&huart2, UartMsg, (uint16_t)UartMsgSize, HAL_MAX_DELAY);
    Radio.Send(TxBuf, TxBufSize);
}
