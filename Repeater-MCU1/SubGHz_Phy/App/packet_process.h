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

#include <stdbool.h>
#include <stdint.h>

#define RECEIVED_PACKET_INFO_FIFO_SIZE 20U

typedef struct
{
  uint16_t packetIDs[RECEIVED_PACKET_INFO_FIFO_SIZE];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
} PacketIDFifo_t;

extern PacketIDFifo_t processedPktBuf;
extern PacketIDFifo_t lowerDistanceDuplicatePktBuf;
extern PacketIDFifo_t higherDistanceDuplicatePktBuf;

bool PacketIDFifo_Push(PacketIDFifo_t *fifo, uint16_t packetID);
bool PacketIDFifo_Pop(PacketIDFifo_t *fifo, uint16_t *packetID);
bool PacketIDFifo_Search(const PacketIDFifo_t *fifo, uint16_t packetID);

#ifdef __cplusplus
}
#endif

#endif /* APP_PACKET_PROCESS_H_ */
