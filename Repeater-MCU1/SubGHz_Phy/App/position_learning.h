/*
 * position_learning.h
 *
 *  Created on: Jun 10, 2026
 *      Author: Nalith
 */

#ifndef APP_POSITION_LEARNING_H_
#define APP_POSITION_LEARNING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "packet.h"
#include "packet_process.h"

extern PacketIDFifo_t repeatedPl1PktIDs;

void PositionLearningInit(void);
void PositionLearningInitialBroadcast(void);
void PL2InitialTransmission(void);
void ReceivedPktHanderPL1(LoRaPacket_t *packet);
void ReceivedPktHanderPL2(LoRaPacket_t *packet);
void ReceivedPktHanderPL3(LoRaPacket_t *packet);

#ifdef __cplusplus
}
#endif

#endif /* APP_POSITION_LEARNING_H_ */
