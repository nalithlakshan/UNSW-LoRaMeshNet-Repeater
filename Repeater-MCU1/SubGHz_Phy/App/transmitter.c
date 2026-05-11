/*
 * transmitter.c
 *
 *  Created on: May 10, 2026
 *      Author: Nalith
 */

#include "transmitter.h"

#include "radio.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "subghz_phy_app.h"
#include "sys_app.h"
#include "usart.h"
#include "utilities_def.h"

#include <stdio.h>

#define TRANSMITTER_PERIOD_MS        8000
#define TRANSMITTER_BUFFER_SIZE      256
#define DEBUG_TX                     1

static UTIL_TIMER_Object_t TxTimer;
static uint8_t TxBuf[TRANSMITTER_BUFFER_SIZE];
static uint16_t TxBufSize = 0;
static uint8_t TxCounter = 0;

static void TxTimerCb(void *context);
static void TxTask(void);

void Transmitter_StartPeriodicED(void)
{
    UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_LoRaTx), 0, TxTask);
    TxBufSize = snprintf((char *)TxBuf, sizeof(TxBuf), "|%d|%d|%c|payload\r\n", nodeID, nextUptreamNodeID, 'U');
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
    uint8_t UartMsg[TRANSMITTER_BUFFER_SIZE] = {0};

    int UartMsgSize = snprintf((char *)UartMsg, sizeof(UartMsg),
                                "Node %d: Transmitting Packet %s\r\n", nodeID, TxBuf);
    HAL_UART_Transmit(&huart2, UartMsg, (uint16_t)UartMsgSize, HAL_MAX_DELAY);
    Radio.Send(TxBuf, TxBufSize);
}
