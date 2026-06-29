/*
 * wor_ack_wait.h
 *
 *  Created on: Jun 24, 2026
 *      Author: Nalith
 */

#ifndef APP_WOR_ACK_WAIT_H_
#define APP_WOR_ACK_WAIT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "packet.h"

#include <stdbool.h>

extern volatile bool firstWorAfterWaking;
extern volatile bool awaitingWorAck;

void WorAckWait_Init(void);
void WorAckWait_HandleReceivedPacket(const LoRaPacket_t *packet);

#ifdef __cplusplus
}
#endif

#endif /* APP_WOR_ACK_WAIT_H_ */
