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
#include <stdlib.h>
#include <string.h>

#define TRANSMITTER_PERIOD_MS        30000
#define WOR_REPLY_WAIT_TIMEOUT_MS    5000U
#define MAX_PACKET_SIZE              256
#define TRANSMITTER_UART_BUFFER_SIZE 512
#define DEBUG_TX                     1

#define TX_TIMEOUT_VALUE             3000
#define TX_BACKOFF_MIN_MS            10U
#define TX_BACKOFF_MAX_MS            500U

TransmitBuffer_t Transmit_Buffer = {0};
bool txLoopRunning = false;
volatile bool edTxCycleActive = false;

typedef enum
{
    ED_TX_STATE_IDLE = 0,
    ED_TX_STATE_TX_WOR,
    ED_TX_STATE_WAIT_WOR_REPLY,
    ED_TX_STATE_REPLY_TIMEOUT,
    ED_TX_STATE_TX_DATA
} EdTxState_t;

static UTIL_TIMER_Object_t TxTimer;
static UTIL_TIMER_Object_t WorReplyTimer;
static uint8_t EncodedTxPkt[MAX_PACKET_SIZE];
static uint16_t EncodedTxPktSize = 0;
static EdTxState_t EdTxState = ED_TX_STATE_IDLE;
static uint8_t SelectedUpstreamNodeType = PACKET_NODE_TYPE_REPEATER;
static bool UnknownUpstreamRetryUsed = false;

static void TxTimerCb(void *context);
static void WorReplyTimerCb(void *context);
static void TxTask(void);
static bool SubmitWorPacket(void);
static bool SubmitDataPacket(void);
static void StartWorAckRx(void);
static void HandleWorReplyTimeout(void);
static void FinishEdCycle(void);

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
        uint32_t channelFreq = (packetToTransmit.packetType == PACKET_TYPE_DATA) ? RF_FREQUENCY_ED_DATA : RF_FREQUENCY_WOR;
        Radio.SetChannel(channelFreq);

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
            if(DEBUG_TX){
                APP_LOG(TS_OFF, VLEVEL_M, "Transmitting packet ID %u\r\n", packetToTransmit.packetID);
            }
            Radio.Send(EncodedTxPkt, EncodedTxPktSize);
        }
    }
    txLoopRunning = false;
}

void Transmitter_Init(void)
{
    UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_LoRaTxLoop), 0, Transmitter_TxLoop);
    srand(HAL_GetTick());
}

void Transmitter_StartPeriodicED(void)
{
    UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_LoRaTx), 0, TxTask);

    UTIL_TIMER_Create(&TxTimer, TRANSMITTER_PERIOD_MS, UTIL_TIMER_PERIODIC, TxTimerCb, NULL);
    UTIL_TIMER_Create(&WorReplyTimer, WOR_REPLY_WAIT_TIMEOUT_MS, UTIL_TIMER_ONESHOT, WorReplyTimerCb, NULL);
    UTIL_TIMER_Start(&TxTimer);
}

void Transmitter_OnTxDone(void)
{
    if (DEBUG_TX)
    {
        APP_LOG(TS_OFF, VLEVEL_M, "TX done\r\n");
    }

    if (EdTxState == ED_TX_STATE_TX_WOR)
    {
        EdTxState = ED_TX_STATE_WAIT_WOR_REPLY;
        UTIL_TIMER_StartWithPeriod(&WorReplyTimer, WOR_REPLY_WAIT_TIMEOUT_MS);
        StartWorAckRx();
        return;
    }

    if (EdTxState == ED_TX_STATE_TX_DATA)
    {
        FinishEdCycle();
        return;
    }

    Radio.Sleep();
}

void Transmitter_OnTxTimeout(void)
{
    if (DEBUG_TX)
    {
        APP_LOG(TS_OFF, VLEVEL_M, "TX timeout\r\n");
    }

    if (EdTxState == ED_TX_STATE_TX_WOR)
    {
        EdTxState = ED_TX_STATE_REPLY_TIMEOUT;
        UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_LoRaTx), CFG_SEQ_Prio_0);
        return;
    }

    FinishEdCycle();
}

void Transmitter_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    LoRaPacket_t packet;

    (void)rssi;
    (void)snr;

    if (EdTxState != ED_TX_STATE_WAIT_WOR_REPLY)
    {
        Radio.Sleep();
        return;
    }

    if ((payload == NULL) || (size < LORA_PACKET_HEADER_SIZE))
    {
        StartWorAckRx();
        return;
    }

    packet = Packet_Decode(payload);
    if ((packet.packetType == PACKET_TYPE_WOR) &&
        (packet.direction == PACKET_DIRECTION_UPSTREAM) &&
        ((packet.txNodeType == PACKET_NODE_TYPE_REPEATER) ||
         (packet.txNodeType == PACKET_NODE_TYPE_GATEWAY)))
    {
        UTIL_TIMER_Stop(&WorReplyTimer);
        Radio.Sleep();
        nextUptreamNodeID = packet.txNodeID;
        nextUptreamNodeDV = packet.txDistanceValue;
        SelectedUpstreamNodeType = packet.txNodeType;
        APP_LOG(TS_OFF, VLEVEL_M, "WOR reply received from node %u\r\n", nextUptreamNodeID);

        if (!SubmitDataPacket())
        {
            APP_LOG(TS_OFF, VLEVEL_M, "DATA submit skipped, transmit buffer full\r\n");
            FinishEdCycle();
        }
        return;
    }

    StartWorAckRx();
}

void Transmitter_OnRxTimeout(void)
{
    if (DEBUG_TX)
    {
        APP_LOG(TS_OFF, VLEVEL_M, "RX timeout\r\n");
    }

    HandleWorReplyTimeout();
}

void Transmitter_OnRxError(void)
{
    if (DEBUG_TX)
    {
        APP_LOG(TS_OFF, VLEVEL_M, "RX Error\r\n");
    }

    if (EdTxState == ED_TX_STATE_WAIT_WOR_REPLY)
    {
        StartWorAckRx();
        return;
    }

    FinishEdCycle();
}


static void TxTimerCb(void *context)
{
    (void)context;
    if ((DEBUG_TX) && (EdTxState == ED_TX_STATE_IDLE))
    {
        APP_LOG(TS_OFF, VLEVEL_M, "TxTimer Expired\r\n");
    }
    if (EdTxState == ED_TX_STATE_IDLE)
    {
        UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_LoRaTx), CFG_SEQ_Prio_0);
    }
}

static void WorReplyTimerCb(void *context)
{
    (void)context;

    HandleWorReplyTimeout();
}

static void TxTask(void)
{
    if (EdTxState == ED_TX_STATE_REPLY_TIMEOUT)
    {
        Radio.Sleep();

        if ((nextUptreamNodeID > 0U) && (!UnknownUpstreamRetryUsed))
        {
            UnknownUpstreamRetryUsed = true;
            nextUptreamNodeID = 0U;
            nextUptreamNodeDV = 0U;
            APP_LOG(TS_OFF, VLEVEL_M, "No WOR reply, retrying with unknown upstream node\r\n");
            if (!SubmitWorPacket())
            {
                APP_LOG(TS_OFF, VLEVEL_M, "WOR retry skipped, transmit buffer full\r\n");
                FinishEdCycle();
            }
        }
        else
        {
            dataPktCounter++;
            APP_LOG(TS_OFF, VLEVEL_M, "No WOR reply, DATA %u discarded\r\n", dataPktCounter);
            FinishEdCycle();
        }

        return;
    }

    if (EdTxState != ED_TX_STATE_IDLE)
    {
        return;
    }

    edTxCycleActive = true;
    UnknownUpstreamRetryUsed = false;
    if (!SubmitWorPacket())
    {
        APP_LOG(TS_OFF, VLEVEL_M, "WOR submit skipped, transmit buffer full\r\n");
        FinishEdCycle();
    }
}

static bool SubmitWorPacket(void)
{
    LoRaPacket_t packet = {0};
    uint8_t nextSequenceNumber = (uint8_t)(sequenceNumber + 1U);

    packet.txNodeID = nodeID;
    packet.txNodeType = PACKET_NODE_TYPE_END_DEVICE;
    packet.txDistanceValue = distanceValue;
    packet.txBatteryPercentage = (uint8_t)batteryPercentage;
    packet.rxNodeID = nextUptreamNodeID;
    packet.rxNodeType = PACKET_NODE_TYPE_REPEATER;
    packet.rxDistanceValue = (nextUptreamNodeID > 0U) ? nextUptreamNodeDV : 0U;
    packet.ackNodeID = 0U;
    packet.nearestGwID = 0U;
    packet.packetID = (uint16_t)(((uint16_t)nodeID << 8) | nextSequenceNumber);
    packet.packetType = PACKET_TYPE_WOR;
    packet.direction = PACKET_DIRECTION_UPSTREAM;
    packet.positionLearningMode = 0U;
    packet.preambleSize = LORA_PREAMBLE_LENGTH;
    packet.payloadSize = 0U;

    EdTxState = ED_TX_STATE_TX_WOR;

    if (Transmitter_Submit(&packet))
    {
        sequenceNumber = nextSequenceNumber;
        APP_LOG(TS_OFF, VLEVEL_M, "Submitted ED WOR: %s\r\n", Packet_To_String(&packet));
        return true;
    }

    EdTxState = ED_TX_STATE_IDLE;
    return false;
}

static bool SubmitDataPacket(void)
{
    LoRaPacket_t packet = {0};
    uint8_t nextSequenceNumber = (uint8_t)(sequenceNumber + 1U);
    uint16_t nextDataPktCounter = (uint16_t)(dataPktCounter + 1U);
    int payloadSize;

    payloadSize = snprintf((char *)packet.payload,
                           sizeof(packet.payload),
                           "DATA from ED %u, number=%u",
                           nodeID,
                           nextDataPktCounter);
    if (payloadSize < 0)
    {
        payloadSize = 0;
    }
    if (payloadSize > LORA_PACKET_MAX_PAYLOAD_SIZE)
    {
        payloadSize = LORA_PACKET_MAX_PAYLOAD_SIZE;
    }

    packet.txNodeID = nodeID;
    packet.txNodeType = PACKET_NODE_TYPE_END_DEVICE;
    packet.txDistanceValue = distanceValue;
    packet.txBatteryPercentage = (uint8_t)batteryPercentage;
    packet.rxNodeID = nextUptreamNodeID;
    packet.rxNodeType = SelectedUpstreamNodeType;
    packet.rxDistanceValue = (nextUptreamNodeID > 0U) ? nextUptreamNodeDV : 0U;
    packet.ackNodeID = 0U;
    packet.nearestGwID = 0U;
    packet.packetID = (uint16_t)(((uint16_t)nodeID << 8) | nextSequenceNumber);
    packet.packetType = PACKET_TYPE_DATA;
    packet.direction = PACKET_DIRECTION_UPSTREAM;
    packet.positionLearningMode = 0U;
    packet.preambleSize = LORA_PREAMBLE_LENGTH;
    packet.payloadSize = (uint16_t)payloadSize;

    EdTxState = ED_TX_STATE_TX_DATA;

    if (Transmitter_Submit(&packet))
    {
        sequenceNumber = nextSequenceNumber;
        dataPktCounter = nextDataPktCounter;
        APP_LOG(TS_OFF, VLEVEL_M, "Submitted ED DATA: %s\r\n", Packet_To_String(&packet));
        return true;
    }

    EdTxState = ED_TX_STATE_IDLE;
    return false;
}

static void StartWorAckRx(void)
{
    if (EdTxState == ED_TX_STATE_WAIT_WOR_REPLY)
    {
        Radio.SetChannel(RF_FREQUENCY_WOR);
        Radio.Rx(WOR_REPLY_WAIT_TIMEOUT_MS);
    }
}

static void HandleWorReplyTimeout(void)
{
    if (EdTxState == ED_TX_STATE_WAIT_WOR_REPLY)
    {
        UTIL_TIMER_Stop(&WorReplyTimer);
        EdTxState = ED_TX_STATE_REPLY_TIMEOUT;
        UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_LoRaTx), CFG_SEQ_Prio_0);
    }
}

static void FinishEdCycle(void)
{
    UTIL_TIMER_Stop(&WorReplyTimer);
    EdTxState = ED_TX_STATE_IDLE;
    UnknownUpstreamRetryUsed = false;
    edTxCycleActive = false;
    Radio.Sleep();
}
