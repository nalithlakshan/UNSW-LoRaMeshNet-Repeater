/*
 * packet_process.h
 *
 *  Created on: Jun 11, 2026
 *      Author: nalit
 */

#ifndef APP_PACKET_PROCESS_H_
#define APP_PACKET_PROCESS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "packet.h"

#include <stdbool.h>
#include <stdint.h>

#define MAX_PACKET_FIFO_SIZE 10U

#define STANDBY_TIMER_MIN_MS 500U
#define STANDBY_TIMER_MAX_MS 3000U
#define WOR_ACK_TO_ED_TIMER_MIN_MS 500U
#define WOR_ACK_TO_ED_TIMER_MAX_MS 3000U

typedef struct
{
  uint16_t packetIDs[MAX_PACKET_FIFO_SIZE];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
} PacketIDFifo_t;

typedef struct
{
  LoRaPacket_t packets[MAX_PACKET_FIFO_SIZE];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
} PacketFifo_t;

extern PacketIDFifo_t processedPktBuf;
extern PacketIDFifo_t lowerDistanceDuplicatePktBuf;
extern PacketIDFifo_t higherDistanceDuplicatePktBuf;
extern PacketIDFifo_t pendingWorAckNodes;
extern PacketFifo_t rxBuffer;
extern PacketFifo_t standbyBuffer;

bool PacketIDFifo_Push(PacketIDFifo_t *fifo, uint16_t packetID);
bool PacketIDFifo_Pop(PacketIDFifo_t *fifo, uint16_t *packetID);
bool PacketIDFifo_Search(const PacketIDFifo_t *fifo, uint16_t packetID);
bool PacketIDFifo_Remove(PacketIDFifo_t *fifo, uint16_t packetID);
void PacketIDFifo_Clear(PacketIDFifo_t *fifo);

bool PacketFifo_Push(PacketFifo_t *fifo, const LoRaPacket_t *packet);
bool PacketFifo_Pop(PacketFifo_t *fifo, LoRaPacket_t *packet);

void PacketProcess_Init(void);
void PacketProcess_Schedule(void);
bool PacketProcess_IsBusy(void);
void PacketProcess_ReconfigureAndSubmit(LoRaPacket_t *packet);

#ifdef __cplusplus
}
#endif

#endif /* APP_PACKET_PROCESS_H_ */
