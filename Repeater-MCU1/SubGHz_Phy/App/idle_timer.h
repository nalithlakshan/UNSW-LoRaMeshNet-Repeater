/*
 * idle_timer.h
 *
 *  Created on: Jun 24, 2026
 *      Author: Nalith
 */

#ifndef APP_IDLE_TIMER_H_
#define APP_IDLE_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

void IdleTimer_Init(void);
void IdleTimer_Reset(void);
void IdleTimer_RestartIfRunning(void);
bool IdleTimer_ShouldEnterLowPower(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_IDLE_TIMER_H_ */
