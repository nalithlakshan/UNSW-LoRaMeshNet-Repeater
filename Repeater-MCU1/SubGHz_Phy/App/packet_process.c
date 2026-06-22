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
#include "stm32_timer.h"
#include "utilities_def.h"
#include "sys_app.h"

#include <stddef.h>
#include <stdlib.h>

PacketIDFifo_t processedPktBuf = {0};
PacketIDFifo_t lowerDistanceDuplicatePktBuf = {0};
PacketIDFifo_t higherDistanceDuplicatePktBuf = {0};
PacketIDFifo_t pendingWorAckNodes = {0};
PacketFifo_t rxBuffer = {0};
PacketFifo_t standbyBuffer = {0};

static void PacketProcess(void);
static void PacketProcess_StandbyTimerCb(void *context);
static bool PacketProcess_StartStandbyTimer(void);
static void PacketProcess_WorAckToEdTimerCb(void *context);
static bool PacketProcess_StartWorAckToEdTimer(const LoRaPacket_t *packet);

static UTIL_TIMER_Object_t StandbyTimers[MAX_PACKET_FIFO_SIZE];
static UTIL_TIMER_Object_t WorAckToEdTimers[MAX_PACKET_FIFO_SIZE];
static LoRaPacket_t worAckToEdPktBuffer[MAX_PACKET_FIFO_SIZE];
static uint8_t WorAckToEdTimerIndexes[MAX_PACKET_FIFO_SIZE];

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

void PacketIDFifo_Clear(PacketIDFifo_t *fifo)
{
  if (fifo == NULL)
  {
    return;
  }

  fifo->head = 0U;
  fifo->tail = 0U;
  fifo->count = 0U;
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
  uint8_t i;

  UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_PacketProcess), 0, PacketProcess);

  for (i = 0U; i < MAX_PACKET_FIFO_SIZE; i++)
  {
    UTIL_TIMER_Create(&StandbyTimers[i],
                      STANDBY_TIMER_MAX_MS,
                      UTIL_TIMER_ONESHOT,
                      PacketProcess_StandbyTimerCb,
                      NULL);

    WorAckToEdTimerIndexes[i] = i;
    UTIL_TIMER_Create(&WorAckToEdTimers[i],
                      WOR_ACK_TO_ED_TIMER_MAX_MS,
                      UTIL_TIMER_ONESHOT,
                      PacketProcess_WorAckToEdTimerCb,
                      &WorAckToEdTimerIndexes[i]);
  }
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
      }

      continue;
    }

    // Packet addressed to this repeater OR Broadcast packet:
    if ((packet.rxNodeID == nodeID) || (packet.direction == PACKET_DIRECTION_BROADCAST))
    {
      PacketProcess_ReconfigureAndSubmit(&packet);

      continue;
    }

    // Standby monitoring candidate from RP/GW:
    if (((packet.txNodeType == PACKET_NODE_TYPE_REPEATER) ||(packet.txNodeType == PACKET_NODE_TYPE_GATEWAY)) &&
        (packet.rxNodeID != 0) &&
        (((packet.direction == PACKET_DIRECTION_UPSTREAM) && (packet.txDistanceValue > distanceValue) && (distanceValue > packet.rxDistanceValue)) ||
         ((packet.direction == PACKET_DIRECTION_DOWNSTREAM) && (packet.txDistanceValue < distanceValue) && (distanceValue < packet.rxDistanceValue))))
    {
      if (PacketProcess_StartStandbyTimer())
      {
        PacketFifo_Push(&standbyBuffer, &packet);
        APP_LOG(TS_OFF, VLEVEL_M, "Submitted packet for standby monitoring: %s\r\n", Packet_To_String(&packet));
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_M, "Standby monitoring skipped, no timer available\r\n");
      }

      continue;
    }

    // ED packet without a selected next node:
    if ((packet.txNodeType == PACKET_NODE_TYPE_END_DEVICE) &&
        (packet.rxNodeID == 0U) &&
        (packet.direction != PACKET_DIRECTION_BROADCAST))
    {
      PacketIDFifo_Push(&pendingWorAckNodes, packet.txNodeID);

      if (PacketProcess_StartWorAckToEdTimer(&packet))
      {
        APP_LOG(TS_OFF, VLEVEL_M, "Started a timer for WOR-ACK to ED: %u\r\n", packet.txNodeID);
      }
      else
      {
        PacketIDFifo_Remove(&pendingWorAckNodes, packet.txNodeID);
        APP_LOG(TS_OFF, VLEVEL_M, "WOR-ACK to ED skipped, no timer available\r\n");
      }

      continue;
    }
  }
}

static bool PacketProcess_StartStandbyTimer(void)
{
  uint8_t i;
  uint32_t delayMs;

  for (i = 0U; i < MAX_PACKET_FIFO_SIZE; i++)
  {
    if (UTIL_TIMER_IsRunning(&StandbyTimers[i]) == 0U)
    {
      delayMs = STANDBY_TIMER_MIN_MS +
                ((uint32_t)rand() % (STANDBY_TIMER_MAX_MS - STANDBY_TIMER_MIN_MS + 1U));
      UTIL_TIMER_StartWithPeriod(&StandbyTimers[i], delayMs);
      return true;
    }
  }

  return false;
}

static void PacketProcess_StandbyTimerCb(void *context)
{
  LoRaPacket_t packet = {0};

  (void)context;

  if (!PacketFifo_Pop(&standbyBuffer, &packet))
  {
    return;
  }

  if ((packet.direction == PACKET_DIRECTION_UPSTREAM) &&
      PacketIDFifo_Search(&lowerDistanceDuplicatePktBuf, packet.packetID))
  {
    APP_LOG(TS_OFF, VLEVEL_M, "Standby packet ignored, lower-distance duplicate seen: %u\r\n", packet.packetID);
    return;
  }

  if ((packet.direction == PACKET_DIRECTION_DOWNSTREAM) &&
      PacketIDFifo_Search(&higherDistanceDuplicatePktBuf, packet.packetID))
  {
    APP_LOG(TS_OFF, VLEVEL_M, "Standby packet ignored, higher-distance duplicate seen: %u\r\n", packet.packetID);
    return;
  }

  PacketProcess_ReconfigureAndSubmit(&packet);
}

static bool PacketProcess_StartWorAckToEdTimer(const LoRaPacket_t *packet)
{
  uint8_t i;
  uint32_t delayMs;

  if (packet == NULL)
  {
    return false;
  }

  for (i = 0U; i < MAX_PACKET_FIFO_SIZE; i++)
  {
    if (UTIL_TIMER_IsRunning(&WorAckToEdTimers[i]) == 0U)
    {
      worAckToEdPktBuffer[i] = *packet;

      delayMs = WOR_ACK_TO_ED_TIMER_MIN_MS +
                ((uint32_t)rand() % (WOR_ACK_TO_ED_TIMER_MAX_MS - WOR_ACK_TO_ED_TIMER_MIN_MS + 1U));

      if (UTIL_TIMER_StartWithPeriod(&WorAckToEdTimers[i], delayMs) == UTIL_TIMER_OK)
      {
        return true;
      }

      return false;
    }
  }

  return false;
}

static void PacketProcess_WorAckToEdTimerCb(void *context)
{
  uint8_t timerIndex;
  LoRaPacket_t packet = {0};

  if (context == NULL)
  {
    return;
  }

  timerIndex = *((uint8_t *)context);
  if (timerIndex >= MAX_PACKET_FIFO_SIZE)
  {
    return;
  }

  packet = worAckToEdPktBuffer[timerIndex];

  if (PacketIDFifo_Search(&pendingWorAckNodes, packet.txNodeID))
  {
    PacketProcess_ReconfigureAndSubmit(&packet);
    PacketIDFifo_Clear(&pendingWorAckNodes);
  }
}

void PacketProcess_ReconfigureAndSubmit(LoRaPacket_t *packet)
{
  if (packet == NULL)
  {
    return;
  }

  packet->txNodeID = nodeID;
  packet->txNodeType = (nodeType == 'E') ? PACKET_NODE_TYPE_END_DEVICE :
                       (nodeType == 'R') ? PACKET_NODE_TYPE_REPEATER :
                                           PACKET_NODE_TYPE_GATEWAY;
  packet->txDistanceValue = distanceValue;
  packet->txBatteryPercentage = (uint8_t)batteryPercentage;
  packet->rxNodeID = (packet->direction == PACKET_DIRECTION_DOWNSTREAM) ? nextDownstreamNodeID :
                     (packet->direction == PACKET_DIRECTION_UPSTREAM) ? nextUptreamNodeID : 0U;
  packet->rxNodeType = ((packet->direction == PACKET_DIRECTION_UPSTREAM) && (packet->rxNodeID != 0) &&
                        (packet->rxNodeID == nearestGatewayID)) ? PACKET_NODE_TYPE_GATEWAY : PACKET_NODE_TYPE_REPEATER;
  packet->rxDistanceValue = 0U; //Edit this later !!!!!!!!!!
  packet->nearestGwID = nearestGatewayID;

  if (Transmitter_Submit(packet))
  {
    APP_LOG(TS_OFF, VLEVEL_M, "Submitted packet for retransmission: %s\r\n", Packet_To_String(packet));
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "Retransmission skipped, transmit buffer full\r\n");
  }
}
