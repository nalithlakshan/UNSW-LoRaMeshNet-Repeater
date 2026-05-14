/*
 * cad_mode.c
 *
 *  Created on: May 8, 2026
 *      Author: Nalith
 */

#include "cad_mode.h"

#include "radio.h"
#include "radio_driver.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "sys_app.h"
#include "utilities_def.h"

#include <stddef.h>
#include <stdint.h>

#define CAD_SCAN_PERIOD_MS           1000
#define CAD_DET_PEAK                 28
#define CAD_DET_MIN                  14
#define CAD_RX_TIMEOUT_VALUE         4000

static UTIL_TIMER_Object_t CadTimer;
static uint32_t CadScanCounter = 0;

static void CadTimerCb(void *context);
static void CAD_Scan(void);

void CAD_Mode_ConfigRadio(void)
{
  SUBGRF_SetCadParams(LORA_CAD_02_SYMBOL, CAD_DET_PEAK, CAD_DET_MIN,
                      LORA_CAD_ONLY, 0);
}

void CAD_Mode_Init(void)
{
  UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_LoRaCadScan), 0, CAD_Scan);
  UTIL_TIMER_Create(&CadTimer, CAD_SCAN_PERIOD_MS, UTIL_TIMER_PERIODIC, CadTimerCb, NULL);
  UTIL_TIMER_Start(&CadTimer);
}

void CAD_Mode_OnCadDone(bool channelActivityDetected)
{
  APP_LOG(TS_OFF, VLEVEL_M, "CAD done: %s\r\n",
          channelActivityDetected ? "activity detected" : "channel clear");

  if (channelActivityDetected == false)
  {
    Radio.Sleep();
  }
  else
  {
    Radio.Rx(CAD_RX_TIMEOUT_VALUE);
  }
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

