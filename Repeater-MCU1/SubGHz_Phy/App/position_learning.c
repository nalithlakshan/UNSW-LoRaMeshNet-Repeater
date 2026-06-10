/*
 * position_learning.c
 *
 *  Created on: Jun 10, 2026
 *      Author: Nalith
 * 
 *  Abbreviations:
 *  - PL: Position Learning
 *  
 */

#include "position_learning.h"

#include "stm32_timer.h"
#include "subghz_phy_app.h"
#include "sys_app.h"

#define MAX_INITIAL_PL_PACKETS 3U
#define INITIAL_PL_BROADCAST_INTERVAL_MS 10000U

static uint8_t initialPktCount = 0;

static UTIL_TIMER_Object_t InitialPlTimer;

static void PositionLearningInitialBroadcastTimerCb(void *context);

void PositionLearningInit(void)
{
  UTIL_TIMER_Create(&InitialPlTimer,
                    INITIAL_PL_BROADCAST_INTERVAL_MS,
                    UTIL_TIMER_ONESHOT,
                    PositionLearningInitialBroadcastTimerCb,
                    NULL);
}

void PositionLearningInitialBroadcast(void)
{
  if (initialPktCount < MAX_INITIAL_PL_PACKETS)
  {
    initialPktCount++;
    UTIL_TIMER_Start(&InitialPlTimer);
  }
  else
  {
    return;
  }
}

static void PositionLearningInitialBroadcastTimerCb(void *context)
{
  (void)context;

  APP_LOG(TS_OFF, VLEVEL_M,
          "RP %u initiating position learning broadcast packet: %u\r\n",
          nodeID,
          initialPktCount);

  PositionLearningInitialBroadcast();
}
