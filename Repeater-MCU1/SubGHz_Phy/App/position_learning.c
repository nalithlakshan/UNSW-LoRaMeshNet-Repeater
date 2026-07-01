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
#include "stm32_seq.h"
#include "sys_app.h"
#include "utilities_def.h"
#include "main.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAX_INITIAL_PL_PACKETS 1U
#define INITIAL_PL_BROADCAST_INTERVAL_MS 3000U
#define PL2_START_TIMEOUT_MS (10000U + nodeID*10000U)
#define PL3_START_TIMEOUT_MS 60000U
#define NETWORK_GRAPH_DISTANCE_INF 0xFFFFFFFFUL
#define DEBUG_PL 1

PacketIDFifo_t repeatedPl1PktIDs = {0};
NetworkGraph_t NetworkGraph = {0};

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
  * This process is executed after PL2 to build a routing table in-house. No LoRa transmission 
  * happens in this phase.
  ******************************************************************************************************
  */

static uint8_t initialPktCount = 0;
static bool positionLearningActive = false;

static UTIL_TIMER_Object_t InitialPlTimer;
static UTIL_TIMER_Object_t PL2StartTimer;
static UTIL_TIMER_Object_t PL3StartTimer;

static void PositionLearningInitialBroadcastTimerCb(void *context);
static void PL2StartTimerCb(void *context);
static void PL3StartTimerCb(void *context);
static bool NetworkGraph_IsRoutingNode(uint8_t nodeType);
static int8_t NetworkGraph_FindNodeIndex(uint8_t nodeID);
static int8_t NetworkGraph_AddOrUpdateNode(uint8_t nodeID, uint8_t nodeType, uint16_t nodeDistanceValue, bool updateDistanceValue);
static void NetworkGraph_SetDirectedLink(uint8_t fromIndex, uint8_t toIndex, uint16_t distance);
static void NetworkGraph_AddLocalNeighbours(void);
static void NetworkGraph_AddPL2Packet(const LoRaPacket_t *packet);
static void NetworkGraph_CalculateShortestPaths(uint8_t sourceIndex, uint32_t distances[NETWORK_GRAPH_MAX_NODES]);
static void NetworkGraph_UpdateLocalRoutingInfo(void);
static void PositionLearning_FormatNeighbourIDs(char *buffer, uint16_t bufferSize);

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

  UTIL_TIMER_Create(&PL3StartTimer,
                    PL3_START_TIMEOUT_MS,
                    UTIL_TIMER_ONESHOT,
                    PL3StartTimerCb,
                    NULL);

  UTIL_SEQ_RegTask((1U << CFG_SEQ_Task_PL3RouteMapping), 0, PL3RouteMapping);
}

void PositionLearningReset(void)
{
  memset(Neighbours, 0, sizeof(Neighbours));
  NeighbourCount = 0U;

  memset(&NetworkGraph, 0, sizeof(NetworkGraph));

  distanceValue = 0U;
  nearestGatewayID = 0U;
  nextUptreamNodeID = 0U;
  nextDownstreamNodeID = 0U;

  // initialPktCount = 0U;
  // PacketIDFifo_Clear(&repeatedPl1PktIDs);
  // PacketIDFifo_Clear(&processedPktBuf);
  // PacketIDFifo_Clear(&lowerDistanceDuplicatePktBuf);
  // PacketIDFifo_Clear(&higherDistanceDuplicatePktBuf);
  // PacketIDFifo_Clear(&upstreamAlreadyForwardedPktBuf);
  // PacketIDFifo_Clear(&pendingWorAckNodes);
}

void PositionLearningInitialBroadcast(void)
{
  positionLearningActive = true;
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); //To indicate PL running
  UTIL_TIMER_StartWithPeriod(&PL2StartTimer, PL2_START_TIMEOUT_MS);

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

  if (!positionLearningActive)
  {
    PositionLearningReset();
    positionLearningActive = true;
  }

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); //To indicate PL running

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
      MQTT_LOG(TS_OFF, VLEVEL_M, "PL1| Node %u retransmitting packet %u/%u\r\n", nodeID, 
        (uint8_t)(packet->packetID >> 8), (uint8_t)packet->packetID);
    }
  }
  else if (DEBUG_PL)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "PL1 repeat skipped, transmit buffer full\r\n");
  }
}

void ReceivedPktHanderPL2(LoRaPacket_t *packet)
{
  UTIL_TIMER_StartWithPeriod(&PL3StartTimer, PL3_START_TIMEOUT_MS);

  NetworkGraph_AddPL2Packet(packet);

  PacketProcess_ReconfigureAndSubmit(packet);

  if(DEBUG_PL){
    MQTT_LOG(TS_OFF, VLEVEL_M, "PL2| Node %u retransmitting packet %u/%u\r\n", nodeID, 
      (uint8_t)(packet->packetID >> 8), (uint8_t)packet->packetID);
  }
}

void PL2InitialTransmission(void)
{
  LoRaPacket_t plPacket = {0};
  uint8_t maxNeighbourCount;
  uint8_t pl2NeighbourCount;
  uint16_t payloadIndex;
  uint8_t i;

  APP_LOG(TS_OFF, VLEVEL_M, "\nPL2 Started!\r\n");
  UTIL_TIMER_StartWithPeriod(&PL3StartTimer, PL3_START_TIMEOUT_MS);

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

  NetworkGraph_AddLocalNeighbours();

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
      MQTT_LOG(TS_OFF, VLEVEL_M, "PL2| Node %u sent packet %u/%u with neighbour table, neighbour count=%u\r\n", 
        nodeID, nodeID, sequenceNumber, pl2NeighbourCount);
    }
  }
  else if (DEBUG_PL)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "PL2 TX skipped, transmit buffer full\r\n");
  }
}

void PL3RouteMapping(void)
{
  uint8_t repeaterIndex;
  uint8_t gatewayIndex;
  uint8_t bestGatewayID;
  uint32_t distances[NETWORK_GRAPH_MAX_NODES];
  uint32_t bestDistance;
  bool gatewayFound;

/* For each repeater in the graph, run Dijkstra from that repeater, find the nearest reachable gateway,
 * and update that repeater's graph record. 
 * Also, if this device is a repeater, update the global variables: distanceValue, nearestGatewayID */
  for (repeaterIndex = 0U; repeaterIndex < NetworkGraph.nodeCount; repeaterIndex++)
  {
    if ((!NetworkGraph.nodes[repeaterIndex].isValid) ||
        (NetworkGraph.nodes[repeaterIndex].nodeType != PACKET_NODE_TYPE_REPEATER))
    {
      continue;
    }

    NetworkGraph_CalculateShortestPaths(repeaterIndex, distances);

    bestGatewayID = 0U;
    bestDistance = NETWORK_GRAPH_DISTANCE_INF;
    gatewayFound = false;

    for (gatewayIndex = 0U; gatewayIndex < NetworkGraph.nodeCount; gatewayIndex++)
    {
      if ((!NetworkGraph.nodes[gatewayIndex].isValid) ||
          (NetworkGraph.nodes[gatewayIndex].nodeType != PACKET_NODE_TYPE_GATEWAY))
      {
        continue;
      }

      if (distances[gatewayIndex] < bestDistance)
      {
        bestDistance = distances[gatewayIndex];
        bestGatewayID = NetworkGraph.nodes[gatewayIndex].nodeID;
        gatewayFound = true;
      }
    }

    if (gatewayFound)
    {
      NetworkGraph.nodes[repeaterIndex].nearestGatewayID = bestGatewayID;
      NetworkGraph.nodes[repeaterIndex].hasRouteToGateway = true;
      NetworkGraph.nodes[repeaterIndex].distanceValue = (bestDistance > 0xFFFFU) ? 0xFFFFU : (uint16_t)bestDistance;

      //If the considered repeater is this device itself
      if (NetworkGraph.nodes[repeaterIndex].nodeID == nodeID)
      {
        nearestGatewayID = bestGatewayID;
        distanceValue = NetworkGraph.nodes[repeaterIndex].distanceValue;
      }

      if (DEBUG_PL)
      {
        APP_LOG(TS_OFF, VLEVEL_M, "PL3| RP %u nearest GW %u, DistanceValue %u\r\n",
                NetworkGraph.nodes[repeaterIndex].nodeID,
                NetworkGraph.nodes[repeaterIndex].nearestGatewayID,
                NetworkGraph.nodes[repeaterIndex].distanceValue);
      }
    }
    else if (DEBUG_PL)
    {
      NetworkGraph.nodes[repeaterIndex].hasRouteToGateway = false;

      APP_LOG(TS_OFF, VLEVEL_M, "PL3| RP %u has no reachable gateway\r\n",
              NetworkGraph.nodes[repeaterIndex].nodeID);
    }
  }


  NetworkGraph_UpdateLocalRoutingInfo();

  positionLearningActive = false;
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  if (DEBUG_PL)
  {
    char neighbourIDs[160];
    PositionLearning_FormatNeighbourIDs(neighbourIDs, sizeof(neighbourIDs));

    MQTT_LOG(TS_OFF, VLEVEL_M,
         "PL_DONE| Node %u GW=%u DV=%u UP=%u DOWN=%u graphNodes=%u neighbours: %s\r\n",
         nodeID, nearestGatewayID, distanceValue,
         nextUptreamNodeID, nextDownstreamNodeID,
         NetworkGraph.nodeCount, neighbourIDs);

    APP_LOG(TS_OFF, VLEVEL_M,
         "PL_DONE| Node %u GW=%u DV=%u UP=%u DOWN=%u graphNodes=%u neighbours: %s\r\n",
         nodeID, nearestGatewayID, distanceValue,
         nextUptreamNodeID, nextDownstreamNodeID,
         NetworkGraph.nodeCount, neighbourIDs);
  }
}

static void PositionLearning_FormatNeighbourIDs(char *buffer, uint16_t bufferSize)
{
  uint8_t i;
  uint16_t used = 0U;

  if ((buffer == NULL) || (bufferSize == 0U))
  {
    return;
  }

  buffer[0] = '\0';

  if (NeighbourCount == 0U)
  {
    (void)snprintf(buffer, bufferSize, "none");
    return;
  }

  for (i = 0U; i < NeighbourCount; i++)
  {
    int written;

    if (used >= bufferSize)
    {
      break;
    }

    written = snprintf(&buffer[used],
                       (uint16_t)(bufferSize - used),
                       "%s%u",
                       (i == 0U) ? "" : ",",
                       Neighbours[i].ID);
    if (written < 0)
    {
      break;
    }

    if ((uint16_t)written >= (uint16_t)(bufferSize - used))
    {
      used = (uint16_t)(bufferSize - 1U);
      break;
    }

    used = (uint16_t)(used + (uint16_t)written);
  }
}

static void PositionLearningInitialBroadcastTimerCb(void *context)
{
  (void)context;

  if(DEBUG_PL){
    APP_LOG(TS_OFF, VLEVEL_M, "\nNode %u initiating position learning broadcast packet: %u\r\n", nodeID, initialPktCount);
    MQTT_LOG(TS_OFF, VLEVEL_M, "\nNode %u initiating position learning broadcast packet: %u\r\n", nodeID, initialPktCount);
  }

  /* Create position learning broadcast packet */
  LoRaPacket_t plPacket = {0};
  plPacket.packetType = PACKET_TYPE_WOR;
  plPacket.packetID = (uint16_t)(((uint16_t)nodeID << 8) | sequenceNumber);
  plPacket.direction = PACKET_DIRECTION_BROADCAST;
  plPacket.txNodeID = nodeID;
  plPacket.txNodeType = (nodeType == 'E') ? PACKET_NODE_TYPE_END_DEVICE :
                       (nodeType == 'R') ? PACKET_NODE_TYPE_REPEATER :
                                           PACKET_NODE_TYPE_GATEWAY;;
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

static void PL3StartTimerCb(void *context)
{
  (void)context;

  UTIL_SEQ_SetTask((1U << CFG_SEQ_Task_PL3RouteMapping), CFG_SEQ_Prio_0);
}

/* Checks whether a node is a routing node (i.e., a repeater/ gateway)
 * Basically this is used to filter End devices which are not routing nodes. 
 */
static bool NetworkGraph_IsRoutingNode(uint8_t nodeType)
{
  return ((nodeType == PACKET_NODE_TYPE_GATEWAY) || (nodeType == PACKET_NODE_TYPE_REPEATER));
}


/* Return graph's index for a given nodeID 
 * If not found --> return -1
 */
static int8_t NetworkGraph_FindNodeIndex(uint8_t graphNodeID)
{
  uint8_t i;

  for (i = 0U; i < NetworkGraph.nodeCount; i++)
  {
    if ((NetworkGraph.nodes[i].isValid) && (NetworkGraph.nodes[i].nodeID == graphNodeID))
    {
      return (int8_t)i;
    }
  }

  return -1;
}


/* Add new node to the NetworkGraph/ Update exisitng node 
 * returns the graph index of the added/updated node
 */
static int8_t NetworkGraph_AddOrUpdateNode(uint8_t graphNodeID, uint8_t graphNodeType, uint16_t graphNodeDistanceValue, bool updateDistanceValue)
{
  int8_t nodeIndex;

  if (!NetworkGraph_IsRoutingNode(graphNodeType))
  {
    return -1;
  }

  // Updating exisiting graph node
  nodeIndex = NetworkGraph_FindNodeIndex(graphNodeID);
  if (nodeIndex >= 0)
  {
    NetworkGraph.nodes[nodeIndex].nodeType = graphNodeType;

    if (graphNodeType == PACKET_NODE_TYPE_GATEWAY)
    {
      NetworkGraph.nodes[nodeIndex].nearestGatewayID = graphNodeID;
      NetworkGraph.nodes[nodeIndex].hasRouteToGateway = true;
      NetworkGraph.nodes[nodeIndex].distanceValue = 0U;
      return nodeIndex;
    }

    if (updateDistanceValue)
    {
      NetworkGraph.nodes[nodeIndex].distanceValue = graphNodeDistanceValue;
    }
    return nodeIndex;
  }

  if (NetworkGraph.nodeCount >= NETWORK_GRAPH_MAX_NODES)
  {
    return -1;
  }

  // Adding a new graph node.
  nodeIndex = (int8_t)NetworkGraph.nodeCount;
  NetworkGraph.nodes[nodeIndex].isValid = true;
  NetworkGraph.nodes[nodeIndex].nodeID = graphNodeID;
  NetworkGraph.nodes[nodeIndex].nodeType = graphNodeType;
  NetworkGraph.nodes[nodeIndex].nearestGatewayID = (graphNodeType == PACKET_NODE_TYPE_GATEWAY) ? graphNodeID : 0U;
  NetworkGraph.nodes[nodeIndex].hasRouteToGateway = (graphNodeType == PACKET_NODE_TYPE_GATEWAY);
  NetworkGraph.nodes[nodeIndex].distanceValue = (graphNodeType == PACKET_NODE_TYPE_GATEWAY) ? 0U : graphNodeDistanceValue;
  NetworkGraph.nodeCount++;

  return nodeIndex;
}


/* Setting a directed link in NetworkGraph with distance as the weight */
static void NetworkGraph_SetDirectedLink(uint8_t fromIndex, uint8_t toIndex, uint16_t distance)
{
  if ((fromIndex >= NetworkGraph.nodeCount) || (toIndex >= NetworkGraph.nodeCount))
  {
    return;
  }

  NetworkGraph.links[fromIndex][toIndex].isValid = true;
  NetworkGraph.links[fromIndex][toIndex].distance = distance;
}


/* Function executed to first add this repeater and its neighbours to the NetworkGraph*/
static void NetworkGraph_AddLocalNeighbours(void)
{
  uint8_t localNodeType;
  int8_t localNodeIndex;
  int8_t neighbourNodeIndex;
  uint8_t i;

  localNodeType = (nodeType == 'E') ? PACKET_NODE_TYPE_END_DEVICE :
                  (nodeType == 'R') ? PACKET_NODE_TYPE_REPEATER :
                                      PACKET_NODE_TYPE_GATEWAY;

  localNodeIndex = NetworkGraph_AddOrUpdateNode(nodeID,
                                                localNodeType,
                                                distanceValue,
                                                (localNodeType == PACKET_NODE_TYPE_GATEWAY));
  if (localNodeIndex < 0)
  {
    return;
  }

  for (i = 0U; i < NeighbourCount; i++)
  {
    if (!NetworkGraph_IsRoutingNode(Neighbours[i].Type))
    {
      continue;
    }

    neighbourNodeIndex = NetworkGraph_AddOrUpdateNode(Neighbours[i].ID,
                                                      Neighbours[i].Type,
                                                      Neighbours[i].DistanceValue,
                                                      (Neighbours[i].Type == PACKET_NODE_TYPE_GATEWAY));
    if (neighbourNodeIndex >= 0)
    {
      NetworkGraph_SetDirectedLink((uint8_t)neighbourNodeIndex, (uint8_t)localNodeIndex, Neighbours[i].Distance);
    }
  }
}


/* Updating distanceValue, nearestGatewayId, nextUpstreamNodeID, nextDownstreamNodeID of
 * this repeater (global variables) and the Neighbours (local records)
 */
static void NetworkGraph_UpdateLocalRoutingInfo(void)
{
  uint8_t i;
  int8_t graphNodeIndex;
  uint8_t localNodeType;
  uint8_t localNearestGatewayID;
  uint16_t localDistanceValue;
  uint16_t lowestDistanceValue = 0xFFFFU;
  uint16_t highestDistanceValue = 0U;
  bool upstreamFound = false;
  bool downstreamFound = false;

  localNodeType = (nodeType == 'E') ? PACKET_NODE_TYPE_END_DEVICE :
                  (nodeType == 'R') ? PACKET_NODE_TYPE_REPEATER :
                                      PACKET_NODE_TYPE_GATEWAY;

  graphNodeIndex = NetworkGraph_FindNodeIndex(nodeID);
  if ((graphNodeIndex >= 0) && (NetworkGraph.nodes[graphNodeIndex].hasRouteToGateway))
  {
    nearestGatewayID = NetworkGraph.nodes[graphNodeIndex].nearestGatewayID;
    distanceValue = NetworkGraph.nodes[graphNodeIndex].distanceValue;
  }

  localNearestGatewayID = nearestGatewayID;
  localDistanceValue = distanceValue;
  nextUptreamNodeID = nodeID;
  nextDownstreamNodeID = nodeID;

  if (localNodeType == PACKET_NODE_TYPE_GATEWAY)
  {
    nearestGatewayID = nodeID;
    nextUptreamNodeID = nodeID;
    localNearestGatewayID = nodeID;
    localDistanceValue = 0U;
    distanceValue = 0U;
  }

  for (i = 0U; i < NeighbourCount; i++)
  {
    graphNodeIndex = NetworkGraph_FindNodeIndex(Neighbours[i].ID);
    if (graphNodeIndex < 0)
    {
      continue;
    }

    Neighbours[i].DistanceValue = NetworkGraph.nodes[graphNodeIndex].distanceValue;

    if ((!NetworkGraph_IsRoutingNode(Neighbours[i].Type)) ||
        (!NetworkGraph.nodes[graphNodeIndex].hasRouteToGateway) ||
        (NetworkGraph.nodes[graphNodeIndex].nearestGatewayID != localNearestGatewayID))
    {
      continue;
    }

    // Updating nextUptreamNodeID
    if ((localNodeType == PACKET_NODE_TYPE_REPEATER) &&
        ((!upstreamFound) || (Neighbours[i].DistanceValue < lowestDistanceValue)))
    {
      lowestDistanceValue = Neighbours[i].DistanceValue;
      nextUptreamNodeID = Neighbours[i].ID;
      upstreamFound = true;
    }

    // Updating nextDownstreamNodeID
    if ((Neighbours[i].DistanceValue > localDistanceValue) &&
        ((!downstreamFound) || (Neighbours[i].DistanceValue > highestDistanceValue)))
    {
      highestDistanceValue = Neighbours[i].DistanceValue;
      nextDownstreamNodeID = Neighbours[i].ID;
      downstreamFound = true;
    }
  }

  if (DEBUG_PL)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "PL3| Local routing: nearest GW %u, DistanceValue %u, upstream %u, downstream %u\r\n",
            nearestGatewayID,
            distanceValue,
            nextUptreamNodeID,
            nextDownstreamNodeID);
  }
}

/* Dijkstra's algorithm:
 * Calculate shortest distances from a given source RP/GW to all routing nodes in the NetworkGraph 
 * The result is put in distances[NETWORK_GRAPH_MAX_NODES] array.
 * i.e., distances[i] =  shortest distance from NetworkGraph.nodes[sourceIndex] to NetworkGraph.nodes[i]
 * For nodes that cannot be reached, distances[i] = NETWORK_GRAPH_DISTANCE_INF
 */
static void NetworkGraph_CalculateShortestPaths(uint8_t sourceIndex, uint32_t distances[NETWORK_GRAPH_MAX_NODES])
{
  bool visited[NETWORK_GRAPH_MAX_NODES] = {0};
  uint8_t i;
  uint8_t neighbourIndex;
  uint8_t selectedIndex;
  uint32_t selectedDistance;
  uint32_t candidateDistance;

  for (i = 0U; i < NETWORK_GRAPH_MAX_NODES; i++)
  {
    distances[i] = NETWORK_GRAPH_DISTANCE_INF;
  }

  if ((sourceIndex >= NetworkGraph.nodeCount) || (!NetworkGraph.nodes[sourceIndex].isValid))
  {
    return;
  }

  distances[sourceIndex] = 0U;

  for (i = 0U; i < NetworkGraph.nodeCount; i++)
  {
    selectedIndex = NETWORK_GRAPH_MAX_NODES;
    selectedDistance = NETWORK_GRAPH_DISTANCE_INF;

    for (neighbourIndex = 0U; neighbourIndex < NetworkGraph.nodeCount; neighbourIndex++)
    {
      if ((!visited[neighbourIndex]) &&
          (NetworkGraph.nodes[neighbourIndex].isValid) &&
          (distances[neighbourIndex] < selectedDistance))
      {
        selectedDistance = distances[neighbourIndex];
        selectedIndex = neighbourIndex;
      }
    }

    if (selectedIndex == NETWORK_GRAPH_MAX_NODES)
    {
      break;
    }

    visited[selectedIndex] = true;

    for (neighbourIndex = 0U; neighbourIndex < NetworkGraph.nodeCount; neighbourIndex++)
    {
      if ((!NetworkGraph.nodes[neighbourIndex].isValid) ||
          (!NetworkGraph.links[selectedIndex][neighbourIndex].isValid) ||
          (distances[selectedIndex] == NETWORK_GRAPH_DISTANCE_INF))
      {
        continue;
      }

      candidateDistance = distances[selectedIndex] + NetworkGraph.links[selectedIndex][neighbourIndex].distance;
      if (candidateDistance < distances[neighbourIndex])
      {
        distances[neighbourIndex] = candidateDistance;
      }
    }
  }
}

static void NetworkGraph_AddPL2Packet(const LoRaPacket_t *packet)
{
  uint8_t deviceID;
  uint8_t deviceType;
  uint8_t neighbourCount;
  uint8_t availableNeighbourCount;
  uint8_t i;
  uint16_t payloadIndex;
  uint8_t neighbourID;
  uint8_t neighbourType;
  uint16_t neighbourDistance;
  int8_t deviceNodeIndex;
  int8_t neighbourNodeIndex;

  if ((packet == NULL) || (packet->payloadSize < 4U) || (packet->payload[0] != 2U))
  {
    return;
  }

  deviceID = packet->payload[1];
  deviceType = packet->payload[2];
  neighbourCount = packet->payload[3];

  if (!NetworkGraph_IsRoutingNode(deviceType))
  {
    return;
  }

  availableNeighbourCount = (uint8_t)((packet->payloadSize - 4U) / 4U);
  if (neighbourCount > availableNeighbourCount)
  {
    neighbourCount = availableNeighbourCount;
  }

  deviceNodeIndex = NetworkGraph_AddOrUpdateNode(deviceID, deviceType, 0U, false);
  if (deviceNodeIndex < 0)
  {
    return;
  }

  payloadIndex = 4U;
  for (i = 0U; i < neighbourCount; i++)
  {
    neighbourID = packet->payload[payloadIndex++];
    neighbourType = packet->payload[payloadIndex++];
    neighbourDistance = (uint16_t)(((uint16_t)packet->payload[payloadIndex] << 8) |
                                   packet->payload[payloadIndex + 1U]);
    payloadIndex += 2U;

    if (!NetworkGraph_IsRoutingNode(neighbourType))
    {
      continue;
    }

    neighbourNodeIndex = NetworkGraph_AddOrUpdateNode(neighbourID, neighbourType, 0U, false);
    if (neighbourNodeIndex >= 0)
    {
      NetworkGraph_SetDirectedLink((uint8_t)neighbourNodeIndex, (uint8_t)deviceNodeIndex, neighbourDistance);
    }
  }
}
