/*
 * idle_timer.c
 *
 *  Created on: Jun 24, 2026
 *      Author: Nalith
 */

#include "idle_timer.h"

#include "main.h"
#include "packet_process.h"
#include "stm32_timer.h"
#include "subghz_phy_app.h"
#include "transmitter.h"
#include "wor_ack_wait.h"

#include <stdint.h>

static UTIL_TIMER_Object_t Mcu1IdleTimer;

void IdleTimer_RestartIfRunning(void)
{
  if (UTIL_TIMER_IsRunning(&Mcu1IdleTimer) != 0U)
  {
    UTIL_TIMER_StartWithPeriod(&Mcu1IdleTimer, MCU1_IDLE_SLEEP_DELAY_MS);
  }
}
static volatile uint8_t Mcu1IdleTimerExpired = 0U;
static volatile uint8_t Mcu1IdleTimerRunning = 0U;
static volatile uint8_t Mcu1IdleTimerResetRequested = 0U;

static void Mcu1IdleTimerCb(void *context);
static void Mcu1IdleTimerResetNow(void);
static bool Mcu1HasPendingWork(void);

void IdleTimer_Init(void)
{
  UTIL_TIMER_Create(&Mcu1IdleTimer,
                    MCU1_IDLE_SLEEP_DELAY_MS,
                    UTIL_TIMER_ONESHOT,
                    Mcu1IdleTimerCb,
                    NULL);
}

void IdleTimer_Reset(void)
{
  Mcu1IdleTimerExpired = 0U;
  Mcu1IdleTimerResetRequested = 1U;
}

bool IdleTimer_ShouldEnterLowPower(void)
{
  if (Mcu1IdleTimerResetRequested != 0U)
  {
    Mcu1IdleTimerResetNow();
  }

  if (Mcu1HasPendingWork())
  {
    Mcu1IdleTimerResetNow();
    return false;
  }

  if (Mcu1IdleTimerExpired == 0U)
  {
    if (Mcu1IdleTimerRunning == 0U)
    {
      if (UTIL_TIMER_StartWithPeriod(&Mcu1IdleTimer, MCU1_IDLE_SLEEP_DELAY_MS) == UTIL_TIMER_OK)
      {
        Mcu1IdleTimerRunning = 1U;
      }
    }

    return false;
  }

  Mcu1IdleTimerExpired = 0U;
  Mcu1IdleTimerRunning = 0U;
  return true;
}

static void Mcu1IdleTimerCb(void *context)
{
  (void)context;
  Mcu1IdleTimerRunning = 0U;
  Mcu1IdleTimerExpired = 1U;
}

static void Mcu1IdleTimerResetNow(void)
{
  if (Mcu1IdleTimerRunning != 0U)
  {
    UTIL_TIMER_Stop(&Mcu1IdleTimer);
  }

  Mcu1IdleTimerRunning = 0U;
  Mcu1IdleTimerExpired = 0U;
  Mcu1IdleTimerResetRequested = 0U;
}

static bool Mcu1HasPendingWork(void)
{
  return (PacketProcess_IsBusy() ||
          Transmitter_IsBusy() ||
          awaitingWorAck ||
          (HAL_GPIO_ReadPin(BTN_GPIO_EXTI9_GPIO_Port, BTN_GPIO_EXTI9_Pin) == GPIO_PIN_RESET) ||
          (HAL_GPIO_ReadPin(WAKE_INT_MCU4_GPIO_Port, WAKE_INT_MCU4_Pin) == GPIO_PIN_SET));
}
