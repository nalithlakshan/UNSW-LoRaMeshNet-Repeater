/*
 * packet_process.c
 *
 *  Created on: Jun 11, 2026
 *      Author: nalit
 */

#include "packet_process.h"

#include <stddef.h>

PacketIDFifo_t processedPktBuf = {0};
PacketIDFifo_t lowerDistanceDuplicatePktBuf = {0};
PacketIDFifo_t higherDistanceDuplicatePktBuf = {0};

bool PacketIDFifo_Push(PacketIDFifo_t *fifo, uint16_t packetID)
{
  if (fifo == NULL)
  {
    return false;
  }

  fifo->packetIDs[fifo->head] = packetID;
  fifo->head = (uint8_t)((fifo->head + 1U) % RECEIVED_PACKET_INFO_FIFO_SIZE);

  if (fifo->count < RECEIVED_PACKET_INFO_FIFO_SIZE)
  {
    fifo->count++;
  }
  else
  {
    fifo->tail = (uint8_t)((fifo->tail + 1U) % RECEIVED_PACKET_INFO_FIFO_SIZE);
  }

  return true;
}

bool PacketIDFifo_Pop(PacketIDFifo_t *fifo, uint16_t *packetID)
{
  if ((fifo == NULL) || (packetID == NULL) || (fifo->count == 0U))
  {
    return false;
  }

  *packetID = fifo->packetIDs[fifo->tail];
  fifo->tail = (uint8_t)((fifo->tail + 1U) % RECEIVED_PACKET_INFO_FIFO_SIZE);
  fifo->count--;

  return true;
}

bool PacketIDFifo_Search(const PacketIDFifo_t *fifo, uint16_t packetID)
{
  uint8_t i;

  if (fifo == NULL)
  {
    return false;
  }

  for (i = 0U; i < fifo->count; i++)
  {
    uint8_t index = (uint8_t)((fifo->tail + i) % RECEIVED_PACKET_INFO_FIFO_SIZE);
    if (fifo->packetIDs[index] == packetID)
    {
      return true;
    }
  }

  return false;
}
