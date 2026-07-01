/*
 * cad_mode.c
 *
 *  Created on: May 8, 2026
 *      Author: Nalith
 */

#include "cad_mode.h"
#include "subghz_phy_app.h"

#include "radio.h"
#include "radio_driver.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "sys_app.h"
#include "utilities_def.h"

#include <stddef.h>
#include <stdint.h>

#define CAD_SCAN_PERIOD_MS           8
#define CAD_DET_PEAK                 28
#define CAD_DET_MIN                  14
#define CAD_RX_TIMEOUT_VALUE         200
#define EXTENDED_CAD_MODE_MS         5000

static UTIL_TIMER_Object_t CadTimer;
static UTIL_TIMER_Object_t ExtendedCadModeTimer;
static uint32_t CadScanCounter = 0;
static bool CadModeActive = false;

static void CadTimerCb(void *context);
static void ExtendedCadModeTimerCb(void *context);
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
  UTIL_TIMER_Create(&ExtendedCadModeTimer, EXTENDED_CAD_MODE_MS, UTIL_TIMER_ONESHOT, ExtendedCadModeTimerCb, NULL);
}

void CAD_Mode_Start(void)
{
  UTIL_TIMER_Stop(&ExtendedCadModeTimer);
  CadModeActive = true;
  UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_LoRaCadScan), CFG_SEQ_Prio_0);
  UTIL_TIMER_Start(&CadTimer);
}

void CAD_Mode_Stop(void)
{
  UTIL_TIMER_Stop(&CadTimer);
  UTIL_TIMER_Stop(&ExtendedCadModeTimer);
  Radio.Sleep();
  CadModeActive = false;
}

void CAD_Mode_StartExtendedStopTimer(void)
{
  if (CadModeActive && (UTIL_TIMER_IsRunning(&ExtendedCadModeTimer) == 0U))
  {
    UTIL_TIMER_StartWithPeriod(&ExtendedCadModeTimer, EXTENDED_CAD_MODE_MS);
  }
}

void CAD_Mode_OnCadDone(bool channelActivityDetected)
{
  // APP_LOG(TS_OFF, VLEVEL_M, "CAD done: %s\r\n", channelActivityDetected ? "activity detected" : "channel clear");

  if (channelActivityDetected){
    APP_LOG(TS_OFF, VLEVEL_M, "CAD activity detected\r\n");
    Radio.Rx(CAD_RX_TIMEOUT_VALUE);
  }
  else{
    Radio.Sleep();
  }
}

static void CadTimerCb(void *context)
{
  (void)context;
  UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_LoRaCadScan), CFG_SEQ_Prio_0);
}

static void ExtendedCadModeTimerCb(void *context)
{
  (void)context;
  CAD_Mode_Stop();
}

static void CAD_Scan(void)
{
  if (Radio.GetStatus() != RF_IDLE)
  {
    // APP_LOG(TS_OFF, VLEVEL_M, "CAD skipped, radio busy (Status: %d)\r\n", Radio.GetStatus());
    return;
  }

  CadScanCounter++;
  // APP_LOG(TS_OFF, VLEVEL_M, "CAD scan #%u\r\n", (unsigned int)CadScanCounter);
  Radio.StartCad();
}

