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
#include "sys_app.h"

#define MAX_INITIAL_PL_PACKETS 3U
#define INITIAL_PL_BROADCAST_INTERVAL_MS 10000U
#define DEBUG_PL 1

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

static void PositionLearningInitialBroadcastTimerCb(void *context);

void PositionLearningInit(void)
{
  UTIL_TIMER_Create(&InitialPlTimer,
                    INITIAL_PL_BROADCAST_INTERVAL_MS,
                    UTIL_TIMER_ONESHOT,
                    PositionLearningInitialBroadcastTimerCb,
                    NULL);
}

void PositionLearningInitialBroadcast(void)
{
  if (initialPktCount < MAX_INITIAL_PL_PACKETS)
  {
    initialPktCount++;
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
}

void ReceivedPktHanderPL2(LoRaPacket_t *packet)
{
}

void ReceivedPktHanderPL3(LoRaPacket_t *packet)
{
}

static void PositionLearningInitialBroadcastTimerCb(void *context)
{
  (void)context;

  if(DEBUG_PL){
    APP_LOG(TS_OFF, VLEVEL_M, "RP %u initiating position learning broadcast packet: %u\r\n", nodeID, initialPktCount);
    MQTT_LOG(TS_OFF, VLEVEL_M, "RP %u initiating position learning broadcast packet: %u\r\n", nodeID, initialPktCount);
  }

  /* Create position learning broadcast packet */
  LoRaPacket_t plPacket = {0};
  plPacket.packetType = PACKET_TYPE_WOR;
  plPacket.packetID = (uint16_t)(((uint16_t)nodeID << 8) | initialPktCount);
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
