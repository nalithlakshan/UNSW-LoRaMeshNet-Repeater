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
#include "subghz_phy_app.h"

#define NETWORK_GRAPH_MAX_NODES 50U // Keep <= 127 since graph indexes use int8_t

typedef struct
{
  bool isValid;
  uint8_t nodeID;
  uint8_t nodeType;
  uint8_t nearestGatewayID;
  bool hasRouteToGateway;
  uint16_t distanceValue;
} NetworkGraphNode_t;

typedef struct
{
  bool isValid;
  uint16_t distance;
} NetworkGraphLink_t;

typedef struct
{
  NetworkGraphNode_t nodes[NETWORK_GRAPH_MAX_NODES];
  NetworkGraphLink_t links[NETWORK_GRAPH_MAX_NODES][NETWORK_GRAPH_MAX_NODES];
  uint8_t nodeCount;
} NetworkGraph_t;

extern PacketIDFifo_t repeatedPl1PktIDs;
extern NetworkGraph_t NetworkGraph;

void PositionLearningInit(void);
void PositionLearningInitialBroadcast(void);
void PL2InitialTransmission(void);
void PL3RouteMapping(void);
void ReceivedPktHanderPL1(LoRaPacket_t *packet);
void ReceivedPktHanderPL2(LoRaPacket_t *packet);

#ifdef __cplusplus
}
#endif

#endif /* APP_POSITION_LEARNING_H_ */
