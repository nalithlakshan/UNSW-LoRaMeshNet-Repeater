/*
 * position_learning.c
 *
 *  Created on: Jun 10, 2026
 *      Author: Nalith
 * 
 *  Abbreviations:
 *  - PL: Position Learning
 *  
 */

#include "position_learning.h"

#include "stm32_timer.h"
#include "subghz_phy_app.h"
#include "packet.h"
#include "transmitter.h"
#include "packet_process.h"
#include "sys_app.h"

#include <math.h>

#define MAX_INITIAL_PL_PACKETS 3U
#define INITIAL_PL_BROADCAST_INTERVAL_MS 10000U
#define PL2_START_TIMEOUT_MS 30000U
#define DEBUG_PL 1

PacketIDFifo_t repeatedPl1PktIDs = {0};

/**
  ******************************************************************************************************
  * When positionLearningMode = 1, the packet's payload can take following formats depending on the
  * position learning phase.
  * 
  * {Phase 1 - Initial Broadcast Phase}
  * |Bytes| Field               | Value/Description
  * |-----|---------------------|----------------------------------------------------------------------
  * | 1   | phase               | value = 1 to indicate initial broadcast phase
  * | 2   | rssi                | RSSI to be filled by receiving node (set to 0 by transmitter)
  * 
  * {Phase 2 - Neighbor Discovery Phase}
  * |Bytes| Field               | Value/Description
  * |-----|---------------------|----------------------------------------------------------------------
  * | 1   | phase               | value = 2 to indicate neighbor discovery phase
  * | 1   | deviceID            | nodeID of the device initiating the packet
  * | 1   | deviceType          | nodeType of the device (0-Gateway, 1-End Device, 2-Repeater)
  * | 1   | neighborCount       | number of neighbors
  * Then for each neighbor:
  * | 1   | neighborID          | nodeID of the neighbor
  * | 1   | neighborType        | nodeType of the neighbor (0-Gateway, 1-End Device, 2-Repeater)
  * | 2   | neighborDistance    | distance to the neighbor from the considered device (in meters)
  * 
  * {Phase 3 - Routing Table Dissemination Phase}
  * |Bytes| Field               | Value/Description
  * |-----|---------------------|----------------------------------------------------------------------
  * | 1   | phase               | value = 3 to indicate routing table dissemination phase
  * | 1   | numberOfRepeaters   | number of repeaters
  * Then for each repeater:
  * | 1   | repeaterID          | nodeID of the repeater
  * | 1   | nearestGw           | nodeID of the nearest gateway to the repeater
  * | 2   | distanceValue       | distance value of the repeater to the nearest gateway
  * | 1   | uptreamNodeID       | nodeID of the next upstream node for this repeater
  * | 1   | downstreamNodeID1   | nodeID of the next downstream node 1 (0 if not applicable)
  * | 1   | downstreamNodeID2   | nodeID of the next downstream node 2 (0 if not applicable)
  * | 1   | downstreamNodeID3   | nodeID of the next downstream node 3 (0 if not applicable)
  ******************************************************************************************************
  */

static uint8_t initialPktCount = 0;

static UTIL_TIMER_Object_t InitialPlTimer;
static UTIL_TIMER_Object_t PL2StartTimer;

static void PositionLearningInitialBroadcastTimerCb(void *context);
static void PL2StartTimerCb(void *context);

void PositionLearningInit(void)
{
  UTIL_TIMER_Create(&InitialPlTimer,
                    INITIAL_PL_BROADCAST_INTERVAL_MS,
                    UTIL_TIMER_ONESHOT,
                    PositionLearningInitialBroadcastTimerCb,
                    NULL);

  UTIL_TIMER_Create(&PL2StartTimer,
                    PL2_START_TIMEOUT_MS,
                    UTIL_TIMER_ONESHOT,
                    PL2StartTimerCb,
                    NULL);
}

void PositionLearningInitialBroadcast(void)
{
  if (initialPktCount < MAX_INITIAL_PL_PACKETS)
  {
    initialPktCount++;
    sequenceNumber++;
    UTIL_TIMER_Start(&InitialPlTimer);
  }
  else
  {
    initialPktCount = 0;
    return;
  }
}

void ReceivedPktHanderPL1(LoRaPacket_t *packet)
{
  //Applying Log Distance Path Loss Model to calculate a distance estimate to the rx node using rssi.
  uint8_t i;
  uint8_t neighbourID;
  int16_t rssi;
  double pl; //path loss
  double distance; //distance to rx node
  double d0 = 5.0; //reference distance (40.0)
  double pl0 = 55; //path loss at d0  (127.41)
  double gamma = 2.08;


  if ((packet == NULL) || (packet->payloadSize < 3U))
  {
    return;
  }

  rssi = (int16_t)(((uint16_t)packet->payload[1] << 8) | packet->payload[2]);
  pl = (double)TX_OUTPUT_POWER - (double)rssi;
  distance = pow(10.0, (pl - pl0)/(10.0 * gamma)) * d0;
  uint32_t distance_x100 = (uint32_t)(distance * 100.0 + 0.5);
  APP_LOG(TS_OFF, VLEVEL_M, "PL1| RSSI = %d, Distance = %u.%02u m\r\n",rssi, (unsigned int)(distance_x100 / 100U),(unsigned int)(distance_x100 % 100U));

  neighbourID = packet->txNodeID;

  for (i = 0U; i < NeighbourCount; i++)
  {
    if (Neighbours[i].ID == neighbourID)
    {
      Neighbours[i].Distance = (uint16_t)distance;
      Neighbours[i].RSSI = rssi;
      break;
    }
  }

  if ((i == NeighbourCount) && (NeighbourCount < MAX_NEIGHBOURS))
  {
    Neighbours[NeighbourCount].ID = neighbourID;
    Neighbours[NeighbourCount].Type = packet->txNodeType;
    Neighbours[NeighbourCount].Distance = (uint16_t)distance;
    Neighbours[NeighbourCount].DistanceValue = 0U; //this is update during position learning phase 3
    Neighbours[NeighbourCount].RSSI = rssi;
    NeighbourCount++;
  }

  UTIL_TIMER_StartWithPeriod(&PL2StartTimer, PL2_START_TIMEOUT_MS);

  if (PacketIDFifo_Search(&repeatedPl1PktIDs, packet->packetID))
  {
    APP_LOG(TS_OFF, VLEVEL_M, "A previously repeated PL1 packet. Skipped!\r\n");
    return;
  }

  packet->payload[1] = 0U;
  packet->payload[2] = 0U;
  packet->txNodeID = nodeID;
  packet->txNodeType = (nodeType == 'E') ? PACKET_NODE_TYPE_END_DEVICE :
                       (nodeType == 'R') ? PACKET_NODE_TYPE_REPEATER :
                                           PACKET_NODE_TYPE_GATEWAY;

  if (Transmitter_Submit(packet))
  {
    PacketIDFifo_Push(&repeatedPl1PktIDs, packet->packetID);

    if (DEBUG_PL)
    {
      APP_LOG(TS_OFF, VLEVEL_M, "Repeatiing PL1 Packet %s\r\n", Packet_To_String(packet));
    }
  }
  else if (DEBUG_PL)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "PL1 repeat skipped, transmit buffer full\r\n");
  }
}

void ReceivedPktHanderPL2(LoRaPacket_t *packet)
{
  (void)packet;
}

void PL2InitialTransmission(void)
{
  LoRaPacket_t plPacket = {0};
  uint8_t maxNeighbourCount;
  uint8_t pl2NeighbourCount;
  uint16_t payloadIndex;
  uint8_t i;

  APP_LOG(TS_OFF, VLEVEL_M, "\nPL2 Started!\r\n");

  maxNeighbourCount = (uint8_t)((LORA_PACKET_MAX_PAYLOAD_SIZE - 4)/4);
  pl2NeighbourCount = (NeighbourCount > maxNeighbourCount) ? maxNeighbourCount : NeighbourCount;

  sequenceNumber++;

  plPacket.packetType = PACKET_TYPE_WOR;
  plPacket.packetID = (uint16_t)(((uint16_t)nodeID << 8) | sequenceNumber);
  plPacket.direction = PACKET_DIRECTION_BROADCAST;
  plPacket.txNodeID = nodeID;
  plPacket.txNodeType = (nodeType == 'E') ? PACKET_NODE_TYPE_END_DEVICE :
                       (nodeType == 'R') ? PACKET_NODE_TYPE_REPEATER :
                                           PACKET_NODE_TYPE_GATEWAY;
  plPacket.positionLearningMode = 1U;
  plPacket.preambleSize = LORA_PREAMBLE_LENGTH;
  plPacket.payloadSize = (uint16_t)(4 + ((uint16_t)pl2NeighbourCount * 4));

  plPacket.payload[0] = 2U;
  plPacket.payload[1] = nodeID;
  plPacket.payload[2] = plPacket.txNodeType;
  plPacket.payload[3] = pl2NeighbourCount;

  payloadIndex = 4;
  for (i = 0U; i < pl2NeighbourCount; i++)
  {
    plPacket.payload[payloadIndex++] = Neighbours[i].ID;
    plPacket.payload[payloadIndex++] = Neighbours[i].Type;
    plPacket.payload[payloadIndex++] = (uint8_t)(Neighbours[i].Distance >> 8);
    plPacket.payload[payloadIndex++] = (uint8_t)Neighbours[i].Distance;
  }

  if (Transmitter_Submit(&plPacket))
  {
    PacketIDFifo_Push(&processedPktBuf, plPacket.packetID);

    if (DEBUG_PL)
    {
      APP_LOG(TS_OFF, VLEVEL_M, "Submitted PL2 Packet %s\r\n", Packet_To_String(&plPacket));
    }
  }
  else if (DEBUG_PL)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "PL2 TX skipped, transmit buffer full\r\n");
  }
}

void ReceivedPktHanderPL3(LoRaPacket_t *packet)
{
}

static void PositionLearningInitialBroadcastTimerCb(void *context)
{
  (void)context;

  if(DEBUG_PL){
    APP_LOG(TS_OFF, VLEVEL_M, "\nRP %u initiating position learning broadcast packet: %u\r\n", nodeID, initialPktCount);
    MQTT_LOG(TS_OFF, VLEVEL_M, "\nRP %u initiating position learning broadcast packet: %u\r\n", nodeID, initialPktCount);
  }

  /* Create position learning broadcast packet */
  LoRaPacket_t plPacket = {0};
  plPacket.packetType = PACKET_TYPE_WOR;
  plPacket.packetID = (uint16_t)(((uint16_t)nodeID << 8) | sequenceNumber);
  plPacket.direction = PACKET_DIRECTION_BROADCAST;
  plPacket.txNodeID = nodeID;
  plPacket.txNodeType = PACKET_NODE_TYPE_REPEATER;
  plPacket.positionLearningMode = 1; // Set position learning mode flag
  plPacket.preambleSize = LORA_PREAMBLE_LENGTH;
  plPacket.payloadSize = 3;
  plPacket.payload[0] = 1; // Phase 1 - Initial Broadcast Phase
  plPacket.payload[1] = 0; // Placeholder for RSSI
  plPacket.payload[2] = 0; // Placeholder for RSSI

  /* Submit the packet for transmission */
  if (Transmitter_Submit(&plPacket))
    {
      // Add plPacket.packetID to processedPktBuf
      PacketIDFifo_Push(&processedPktBuf, plPacket.packetID);
      PacketIDFifo_Push(&repeatedPl1PktIDs, plPacket.packetID); 
      if (DEBUG_PL)
      {
        const char *packetString = Packet_To_String(&plPacket);
        APP_LOG(TS_OFF, VLEVEL_M, "Node %d: Submitted Position Learning Broadcast Packet %s\r\n", nodeID, packetString);
      }
    }
    else if (DEBUG_PL)
    {
        APP_LOG(TS_OFF, VLEVEL_M, "TX skipped, transmit buffer full\r\n");
    }

  PositionLearningInitialBroadcast();
}

static void PL2StartTimerCb(void *context)
{
  (void)context;

  PL2InitialTransmission();
}
