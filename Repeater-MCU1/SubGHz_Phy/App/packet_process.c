/*
 * packet_process.c
 *
 *  Created on: Jun 11, 2026
 *      Author: nalit
 */

#include "packet_process.h"
#include "position_learning.h"
#include "subghz_phy_app.h"
#include "transmitter.h"

#include "stm32_seq.h"
#include "utilities_def.h"
#include "sys_app.h"

#include <stddef.h>

PacketIDFifo_t processedPktBuf = {0};
PacketIDFifo_t lowerDistanceDuplicatePktBuf = {0};
PacketIDFifo_t higherDistanceDuplicatePktBuf = {0};
PacketIDFifo_t pendingWorAckNodes = {0};
PacketFifo_t rxBuffer = {0};

static void PacketProcess(void);

/*-------------------------------------------------------------------------------------
 * Packet ID FIFO Functions: Push, Pop, Search, Remove, Clear 
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

bool PacketIDFifo_Remove(PacketIDFifo_t *fifo, uint16_t packetID)
{
  uint8_t i;
  uint8_t writeIndex;

  if ((fifo == NULL) || (fifo->count == 0U))
  {
    return false;
  }

  for (i = 0U; i < fifo->count; i++)
  {
    uint8_t readIndex = (uint8_t)((fifo->tail + i) % MAX_PACKET_FIFO_SIZE);
    if (fifo->packetIDs[readIndex] == packetID)
    {
      break;
    }
  }

  if (i == fifo->count)
  {
    return false;
  }

  writeIndex = (uint8_t)((fifo->tail + i) % MAX_PACKET_FIFO_SIZE);
  for (; i < (uint8_t)(fifo->count - 1U); i++)
  {
    uint8_t readIndex = (uint8_t)((fifo->tail + i + 1U) % MAX_PACKET_FIFO_SIZE);
    fifo->packetIDs[writeIndex] = fifo->packetIDs[readIndex];
    writeIndex = readIndex;
  }

  fifo->head = (fifo->head == 0U) ? (MAX_PACKET_FIFO_SIZE - 1U) : (uint8_t)(fifo->head - 1U);
  fifo->count--;

  return true;
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

/* PACKET PROCESSING FUNCTION
 * sequencer function that processes received packets from rxBuffer. logic brief:
 * 
 * 1. Check if positionLearningMode is enabled.
 *    1.1 if payload's first byte ==1, it is a PL1 packet, call ReceivedPktHanderPL1(packet)
 *    1.2 if payload's first byte ==2, it is a PL2 packet, call ReceivedPktHanderPL2(packet)
 *    1.3 if payload's first byte ==3, it is a PL3 packet, call ReceivedPktHanderPL3(packet)
 * 
 * 2. else check if it is packet addressed to this repeater or whether it is a broadcast packet. 
 *    If so re-configure and submit it for retransmission.
 * 
 * 3. else check if it is transmitted by a RP/GW and qualifies for standby monitoring 
 *    (i.e. a upstream packet with sender's DV > this repeater's DV > intended receiver's DV
 *    OR a downstream packet with sender's DV < this repeater's DV < intended receiver's DV). 
 *    If so, submit it for standby monitoring by pushing the packet into standbyBuffer and 
 *    starting the standby timer. When timer expires, pop a packet from standbyBuffer and check 
 *    if its packetID is in lowerDistanceDuplicatePktBuf (if an upstream packet) or 
 *    higherDistanceDuplicatePktBuf (if a downstream packet). If so, ignore the packet. If not, 
 *    re-configure and submit it for retransmission.
 * 
 * 4. else check if it is transmitted by an ED without addressing a next node while also not 
 *    being a broadcast packet. If so, append the ED's nodeID to pendingWorAckNodes list and 
 *    start a randomized timer. After timer expires, if the ED's id is still in the list, 
 *    re-configure the packet for retransmission and clear the pendingWorAckNodes list.
**/
static void PacketProcess(void)
{
  LoRaPacket_t packet = {0};

  while (PacketFifo_Pop(&rxBuffer, &packet))
  {
    APP_LOG(TS_OFF, VLEVEL_M, "Processing packet: %s\r\n", Packet_To_String(&packet));

    //Position Learning Mode:
    if (packet.positionLearningMode == 1U)
    {
      if (packet.payloadSize > 0U)
      {
        if (packet.payload[0] == 1U)
        {
          ReceivedPktHanderPL1(&packet);
        }
        else if (packet.payload[0] == 2U)
        {
          ReceivedPktHanderPL2(&packet);
        }
        else if (packet.payload[0] == 3U)
        {
          ReceivedPktHanderPL3(&packet);
        }
      }

      continue;
    }

    // Packet addressed to this repeater OR Broadcast packet:
    if ((packet.rxNodeID == nodeID) || (packet.direction == PACKET_DIRECTION_BROADCAST))
    {
      packet.txNodeID = nodeID;
      packet.txNodeType = (nodeType == 'E') ? PACKET_NODE_TYPE_END_DEVICE :
                          (nodeType == 'R') ? PACKET_NODE_TYPE_REPEATER :
                                              PACKET_NODE_TYPE_GATEWAY;
      packet.txDistanceValue = distanceValue;
      packet.txBatteryPercentage = (uint8_t)batteryPercentage;
      packet.rxNodeID = (packet.direction == PACKET_DIRECTION_DOWNSTREAM)? nextDownstreamNodeID:
                        (packet.direction == PACKET_DIRECTION_UPSTREAM)? nextUptreamNodeID: 0U;
      packet.rxNodeType = ((packet.direction == PACKET_DIRECTION_UPSTREAM) &&
                           (packet.rxNodeID == nearestGatewayID)) ? PACKET_NODE_TYPE_GATEWAY : PACKET_NODE_TYPE_REPEATER;
      packet.rxDistanceValue = 0U;
      packet.nearestGwID = nearestGatewayID;

      if (Transmitter_Submit(&packet))
      {
        APP_LOG(TS_OFF, VLEVEL_M, "Submitted packet for retransmission: %s\r\n", Packet_To_String(&packet));
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_M, "Retransmission skipped, transmit buffer full\r\n");
      }

      continue;
    }

  }
}
