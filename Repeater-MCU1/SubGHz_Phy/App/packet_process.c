/*
 * packet_process.c
 *
 *  Created on: Jun 11, 2026
 *      Author: nalit
 */

#include "packet_process.h"

#include "stm32_seq.h"
#include "utilities_def.h"
#include "sys_app.h"

#include <stddef.h>

PacketIDFifo_t processedPktBuf = {0};
PacketIDFifo_t lowerDistanceDuplicatePktBuf = {0};
PacketIDFifo_t higherDistanceDuplicatePktBuf = {0};
PacketFifo_t rxBuffer = {0};

static void PacketProcess(void);

/*-------------------------------------------------------------------------------------
 * Packet ID FIFO Functions: Push, Pop, Search 
 * Note: These will be used to track received new/duplicate packet IDs in processedPktBuf, 
 * lowerDistanceDuplicatePktBuf, and higherDistanceDuplicatePktBuf.
 *-------------------------------------------------------------------------------------*/
bool PacketIDFifo_Push(PacketIDFifo_t *fifo, uint16_t packetID)
{
  if (fifo == NULL)
  {
    return false;
  }

  fifo->packetIDs[fifo->head] = packetID;
  fifo->head = (uint8_t)((fifo->head + 1U) % MAX_PACKET_FIFO_SIZE);

  if (fifo->count < MAX_PACKET_FIFO_SIZE)
  {
    fifo->count++;
  }
  else
  {
    fifo->tail = (uint8_t)((fifo->tail + 1U) % MAX_PACKET_FIFO_SIZE);
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
  fifo->tail = (uint8_t)((fifo->tail + 1U) % MAX_PACKET_FIFO_SIZE);
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
    uint8_t index = (uint8_t)((fifo->tail + i) % MAX_PACKET_FIFO_SIZE);
    if (fifo->packetIDs[index] == packetID)
    {
      return true;
    }
  }

  return false;
}

/*-------------------------------------------------------------------------------------
 * Packet FIFO Functions: Push, Pop
 * Note: These will be used to queue LoRaPacket_t objects for processing
 *-------------------------------------------------------------------------------------*/
bool PacketFifo_Push(PacketFifo_t *fifo, const LoRaPacket_t *packet)
{
  if ((fifo == NULL) || (packet == NULL))
  {
    return false;
  }

  fifo->packets[fifo->head] = *packet;
  fifo->head = (uint8_t)((fifo->head + 1U) % MAX_PACKET_FIFO_SIZE);

  if (fifo->count < MAX_PACKET_FIFO_SIZE)
  {
    fifo->count++;
  }
  else
  {
    fifo->tail = (uint8_t)((fifo->tail + 1U) % MAX_PACKET_FIFO_SIZE);
  }

  return true;
}

bool PacketFifo_Pop(PacketFifo_t *fifo, LoRaPacket_t *packet)
{
  if ((fifo == NULL) || (packet == NULL) || (fifo->count == 0U))
  {
    return false;
  }

  *packet = fifo->packets[fifo->tail];
  fifo->tail = (uint8_t)((fifo->tail + 1U) % MAX_PACKET_FIFO_SIZE);
  fifo->count--;

  return true;
}

/*-------------------------------------------------------------------------------------
 * PACKET PROCESSING
 *-------------------------------------------------------------------------------------*/

//Packet Processing Sequencer Task Initialization
void PacketProcess_Init(void)
{
  UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_PacketProcess), 0, PacketProcess);
}

//Packet Processing Sequencer Task Scheduler
void PacketProcess_Schedule(void)
{
  UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_PacketProcess), CFG_SEQ_Prio_0);
}

/*
 *Received Packet Processing Function (Main Repeater Logic)
**/
static void PacketProcess(void)
{
  LoRaPacket_t packet = {0};

  while (PacketFifo_Pop(&rxBuffer, &packet))
  {
    APP_LOG(TS_OFF, VLEVEL_M, "Processing packet: %s\r\n", Packet_To_String(&packet));
  }
}
