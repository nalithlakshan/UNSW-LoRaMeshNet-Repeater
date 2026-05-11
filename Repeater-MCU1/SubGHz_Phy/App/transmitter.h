/*
 * transmitter.h
 *
 *  Created on: May 10, 2026
 *      Author: Nalith
 */

#ifndef APP_TRANSMITTER_H_
#define APP_TRANSMITTER_H_

#ifdef __cplusplus
extern "C" {
#endif

void Transmitter_StartPeriodicED(void);
void Transmitter_OnTxDone(void);
void Transmitter_OnTxTimeout(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TRANSMITTER_H_ */
