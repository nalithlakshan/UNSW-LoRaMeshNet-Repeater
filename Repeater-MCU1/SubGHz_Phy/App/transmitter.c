/*
 * transmitter.c
 *
 *  Created on: May 10, 2026
 *      Author: Nalith
 */

#include "transmitter.h"

#include "cad_mode.h"
#include "idle_timer.h"
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
#include <stdlib.h>
#include <string.h>

#define TRANSMITTER_PERIOD_MS        3000
#define MAX_PACKET_SIZE              256
#define TRANSMITTER_UART_BUFFER_SIZE 512
#define DEBUG_TX                     1

#define TX_TIMEOUT_VALUE             3000
#define TX_BACKOFF_MIN_MS            50U
#define TX_BACKOFF_MAX_MS            1000U

TransmitBuffer_t Transmit_Buffer = {0};
bool txLoopRunning = false;
static volatile bool txInProgress = false;


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
        IdleTimer_Reset();
        UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_LoRaTxLoop), CFG_SEQ_Prio_0);
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

        // Setting the channel based on packet type
        uint32_t channelFreq = (packetToTransmit.packetType == PACKET_TYPE_DATA) ? RF_FREQUENCY_RP_DATA : RF_FREQUENCY_WOR;
        uint16_t txPreambleLength = (channelFreq == RF_FREQUENCY_WOR) ? LORA_PREAMBLE_LENGTH_WOR : LORA_PREAMBLE_LENGTH_DATA;
        int8_t txOutputPower = (packetToTransmit.packetType == PACKET_TYPE_DATA) ? TX_OUTPUT_POWER_DATA : TX_OUTPUT_POWER_WOR;
        Radio.SetChannel(channelFreq);
        Radio.SetTxConfig(MODEM_LORA, txOutputPower, 0, LORA_BANDWIDTH,
                          LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                          txPreambleLength, LORA_FIX_LENGTH_PAYLOAD_ON,
                          true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

        // Encode the packet into a byte buffer for transmission
        EncodedTxPktSize = Packet_Encode(&packetToTransmit, EncodedTxPkt, sizeof(EncodedTxPkt));

        // If encoding was successful, perform carrier sense and transmit
        if (EncodedTxPktSize > 0U)
        {
            bool channelFree = false;

            // LoRa carrier Sense with random backoff
            while (!channelFree || (Radio.GetStatus() != RF_IDLE))
            {
                channelFree = false;
                uint32_t backoffMs = TX_BACKOFF_MIN_MS + ((uint32_t)rand() % (TX_BACKOFF_MAX_MS - TX_BACKOFF_MIN_MS + 1U));
                HAL_Delay(backoffMs);

                if (Radio.GetStatus() != RF_IDLE){
                    if(DEBUG_TX){
                        APP_LOG(TS_OFF, VLEVEL_M, "Radio busy (Status: %d), backing off!\r\n", Radio.GetStatus());
                    }
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

                channelFree = (!cadActivityDetected && cadResultReady);
                if(!channelFree && DEBUG_TX){
                    APP_LOG(TS_OFF, VLEVEL_M, "Channel is Busy!\r\n");
                }       
            }
            
            // Transmitting the Packet 
            if(DEBUG_TX)
            {
                APP_LOG(TS_OFF, VLEVEL_M, "Transmitting packet ID %u/%u\r\n",
                        (uint8_t)(packetToTransmit.packetID >> 8),
                        (uint8_t)packetToTransmit.packetID);

                if (packetToTransmit.packetType == PACKET_TYPE_DATA)
                {
                    MQTT_LOG(TS_OFF, VLEVEL_M, "%c%u re-transmitting DATA packet: %u/%u\r\n",
                            nodeType,
                            nodeID,
                            (uint8_t)(packetToTransmit.packetID >> 8),
                            (uint8_t)packetToTransmit.packetID);
                }
                else if ((packetToTransmit.packetType == PACKET_TYPE_WOR) &&
                         (packetToTransmit.positionLearningMode == 0U))
                {
                    MQTT_LOG(TS_OFF, VLEVEL_M, "%c%u re-transmitting WOR packet: %u/%u\r\n",
                            nodeType,
                            nodeID,
                            (uint8_t)(packetToTransmit.packetID >> 8),
                            (uint8_t)packetToTransmit.packetID);
                }
            }
            txInProgress = true;
            Radio.Send(EncodedTxPkt, EncodedTxPktSize);
        }
    }
    txLoopRunning = false;
}

bool Transmitter_IsBusy(void)
{
    return (txLoopRunning || txInProgress || (Transmit_Buffer.count > 0U));
}

void Transmitter_Init(void)
{
    UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_LoRaTxLoop), 0, Transmitter_TxLoop);
    srand(HAL_GetTick());
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
    TxPacket.packetType = PACKET_TYPE_WOR;
    TxPacket.direction = PACKET_DIRECTION_UPSTREAM;
    TxPacket.preambleSize = LORA_PREAMBLE_LENGTH_WOR;
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
    txInProgress = false;
    Radio.SetChannel(RF_FREQUENCY);
    Radio.Sleep();
}

void Transmitter_OnTxTimeout(void)
{
    if (DEBUG_TX)
    {
        APP_LOG(TS_OFF, VLEVEL_M, "TX timeout\r\n");
    }
    txInProgress = false;
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
