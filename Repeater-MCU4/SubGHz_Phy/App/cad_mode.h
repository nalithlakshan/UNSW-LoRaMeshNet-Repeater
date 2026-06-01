/*
 * cad_mode.h
 *
 *  Created on: May 8, 2026
 *      Author: Nalith
 */

#ifndef APP_CAD_MODE_H_
#define APP_CAD_MODE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

void CAD_Mode_ConfigRadio(void);
void CAD_Mode_Init(void);
void CAD_Mode_OnCadDone(bool channelActivityDetected);


#ifdef __cplusplus
}
#endif

#endif /* APP_CAD_MODE_H_ */
