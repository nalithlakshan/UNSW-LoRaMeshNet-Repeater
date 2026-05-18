/*
 * transmitter.c
 *
 *  Created on: May 10, 2026
 *      Author: Nalith
 */

#include "transmitter.h"

#include "cad_mode.h"
#include "packet.h"
#include "radio.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "subghz_phy_app.h"
#include "sys_app.h"
#include "usart.h"
#include "utilities_conf.h"
#include "utilities_def.h"

#include <stdio.h>
#include <string.h>

#define TRANSMITTER_PERIOD_MS        2000
#define MAX_PACKET_SIZE              256
#define TRANSMITTER_UART_BUFFER_SIZE 512
#define DEBUG_TX                     1
#define TX_TIMEOUT_VALUE             3000
#define TX_BACKOFF_MIN_MS            10U
#define TX_BACKOFF_MAX_MS            50U

TransmitBuffer_t Transmit_Buffer = {0};
bool txLoopRunning = false;


static UTIL_TIMER_Object_t TxTimer;
static LoRaPacket_t TxPacket;
static uint8_t EncodedTxPkt[MAX_PACKET_SIZE];
static uint16_t EncodedTxPktSize = 0;
static uint8_t TxCounter = 0;

static void TxTimerCb(void *context);
static void TxTask(void);

bool Transmitter_Submit(const LoRaPacket_t *packet)
{
    bool submitted = false;
    bool startTxLoop = false;

    if (packet == NULL)
    {
        return false;
    }

    UTILS_ENTER_CRITICAL_SECTION();
    if (Transmit_Buffer.count < TRANSMIT_BUFFER_MAX_PACKETS)
    {
        Transmit_Buffer.packets[Transmit_Buffer.head] = *packet;
        Transmit_Buffer.head = (uint8_t)((Transmit_Buffer.head + 1U) % TRANSMIT_BUFFER_MAX_PACKETS);
        Transmit_Buffer.count++;
        submitted = true;

        if(!txLoopRunning){
            txLoopRunning = true;
            startTxLoop = true;
        }
    }
    UTILS_EXIT_CRITICAL_SECTION();

    if(startTxLoop){
        Transmitter_TxLoop();
    }


    return submitted;
}

void Transmitter_TxLoop(void)
{
    while (true)
    {
        LoRaPacket_t packetToTransmit = {0};
        bool packetAvailable = false;

        UTILS_ENTER_CRITICAL_SECTION();
        if (Transmit_Buffer.count > 0U)
        {
            packetToTransmit = Transmit_Buffer.packets[Transmit_Buffer.tail];
            Transmit_Buffer.tail = (uint8_t)((Transmit_Buffer.tail + 1U) % TRANSMIT_BUFFER_MAX_PACKETS);
            Transmit_Buffer.count--;
            packetAvailable = true;
        }
        UTILS_EXIT_CRITICAL_SECTION();

        if (!packetAvailable)
        {
            break;
        }

        EncodedTxPktSize = Packet_Encode(&packetToTransmit, EncodedTxPkt, sizeof(EncodedTxPkt));
        if (EncodedTxPktSize > 0U)
        {
            bool channelFree = false;

            // LoRa carrier Sense with random backoff
            while (!channelFree || (Radio.GetStatus() != RF_IDLE))
            {
                channelFree = false;
                uint32_t backoffMs = TX_BACKOFF_MIN_MS +(Radio.Random() % (TX_BACKOFF_MAX_MS - TX_BACKOFF_MIN_MS + 1U));
                HAL_Delay(backoffMs);

                if (Radio.GetStatus() != RF_IDLE){
                    continue;
                }

                cadResultReady = false;
                Radio.StartCad();
                uint32_t cadWaitMs = 0;
                while (!cadResultReady && (cadWaitMs < 2000U))
                {
                    HAL_Delay(1);
                    cadWaitMs++;
                }

                channelFree = !cadActivityDetected;
                if(!channelFree){
                    APP_LOG(TS_OFF, VLEVEL_M, "Channel is Busy!\r\n");
                }       
            }
            
            // Transmitting the Packet                  
            Radio.Send(EncodedTxPkt, EncodedTxPktSize);
        }
    }
    txLoopRunning = false;
}

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

    EncodedTxPktSize = Packet_Encode(&TxPacket, EncodedTxPkt, sizeof(EncodedTxPkt));

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
    TxCounter = TxCounter + 1U;
    if (DEBUG_TX)
    {
        APP_LOG(TS_OFF, VLEVEL_M, "TX counter = %u\r\n", TxCounter);
    }
    uint8_t UartMsg[TRANSMITTER_UART_BUFFER_SIZE] = {0};
    const char *packetString;

    TxPacket.packetID = (uint16_t)(((uint16_t)nodeID << 8) | TxCounter);
    if (Transmitter_Submit(&TxPacket))
    {
        packetString = Packet_To_String(&TxPacket);
        int UartMsgSize = snprintf((char *)UartMsg, sizeof(UartMsg),
                                   "Node %d: Submitted Packet %s\r\n", nodeID, packetString);
        HAL_UART_Transmit(&huart2, UartMsg, (uint16_t)UartMsgSize, HAL_MAX_DELAY);
    }
    else if (DEBUG_TX)
    {
        APP_LOG(TS_OFF, VLEVEL_M, "TX skipped, transmit buffer full\r\n");
    }
}
