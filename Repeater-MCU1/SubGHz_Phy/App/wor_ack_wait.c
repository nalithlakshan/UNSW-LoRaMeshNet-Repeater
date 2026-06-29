/*
 * wor_ack_wait.c
 *
 *  Created on: Jun 24, 2026
 *      Author: Nalith
 */

#include "wor_ack_wait.h"

#include "packet_process.h"
#include "stm32_timer.h"
#include "subghz_phy_app.h"
#include "sys_app.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define WOR_ACK_WAIT_TIMEOUT_MS 3000U

volatile bool awaitingWorAck = false;
static UTIL_TIMER_Object_t WorAckWaitTimer;
volatile bool firstWorAfterWaking = false;
static uint16_t awaitingWorPacketID = 0U;
static uint8_t awaitingWorDirection = PACKET_DIRECTION_BROADCAST;

static void WorAckWait_TimerCb(void *context);
static bool WorAckWait_ShouldStartWaiting(const LoRaPacket_t *packet);
static bool WorAckWait_ShouldClearWaiting(const LoRaPacket_t *packet);


/* Init: Creates a Timer to check for a WOR-ACK */
void WorAckWait_Init(void)
{
  UTIL_TIMER_Create(&WorAckWaitTimer,
                    WOR_ACK_WAIT_TIMEOUT_MS,
                    UTIL_TIMER_ONESHOT,
                    WorAckWait_TimerCb,
                    NULL);
}


/* Packet handler managing the task of holding DATA packets without forwarding until
 * getting a WOR-ACK from forthcoming nodes to be certain that the path ahead is awake.
 * Called for all received packets in HAL_I2C_SlaveRxCpltCallback
 */
void WorAckWait_HandleReceivedPacket(const LoRaPacket_t *packet)
{
  bool firstWorPacket = false; //first received after waking from sleep?

  if (packet == NULL)
  {
    return;
  }

  // Ignore DATA packets (Only WOR packets considered)
  if (packet->packetType != PACKET_TYPE_WOR)
  {
    return;
  }

  // Ignore Position-learning WOR packets
  if((packet->positionLearningMode == 1U) &&
     (packet->payloadSize > 0U) &&
     ((packet->payload[0] == 1U) || (packet->payload[0] == 2U)))
  {
    return;
  }

  // Ignore WOR packets transmitted by a RP/GW with different nearestGatewayID
  if((packet->txNodeType != PACKET_NODE_TYPE_END_DEVICE) &&
     (packet->nearestGwID != nearestGatewayID))
  {
    return;
  }

  // Checking if this is the first WOR received after waking from sleep mode
  // firstWorAfterWaking flag is set true at UTIL_SEQ_Idle() in sys_app.c
  if (firstWorAfterWaking)
  {
    firstWorPacket = true;
    firstWorAfterWaking = false;
  }

  // Clearing awaitingWorAck flag
  if ((!firstWorPacket) && WorAckWait_ShouldClearWaiting(packet))
  {
    APP_LOG(TS_OFF, VLEVEL_M, "WOR ACK wait cleared by packet ID %u\r\n", packet->packetID);
    UTIL_TIMER_Stop(&WorAckWaitTimer);
    awaitingWorAck = false;
    awaitingWorPacketID = 0U;
    awaitingWorDirection = PACKET_DIRECTION_BROADCAST;
    PacketProcess_ReleaseAwaitingWorData();
    return;
  }

  // Setting awaitingWorAck flag and its timer
  if (firstWorPacket && WorAckWait_ShouldStartWaiting(packet))
  {
    APP_LOG(TS_OFF, VLEVEL_M, "WOR ACK wait started for packet ID %u\r\n", packet->packetID);
    awaitingWorAck = true;
    awaitingWorPacketID = packet->packetID;
    awaitingWorDirection = packet->direction;
    UTIL_TIMER_StartWithPeriod(&WorAckWaitTimer, WOR_ACK_WAIT_TIMEOUT_MS);
  }
}


/* Timeout is the fail-safe path: if no matching WOR retransmission is heard,
 * clear awaitingWorAck flag and allow held DATA to be forwarded anyway.
 */
static void WorAckWait_TimerCb(void *context)
{
  (void)context;

  if (awaitingWorAck)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "WOR ACK wait timed out for packet ID %u\r\n", awaitingWorPacketID);
    UTIL_TIMER_Stop(&WorAckWaitTimer);
    awaitingWorAck = false;
    awaitingWorPacketID = 0U;
    awaitingWorDirection = PACKET_DIRECTION_BROADCAST;
    PacketProcess_ReleaseAwaitingWorData();
  }
}


static bool WorAckWait_ShouldStartWaiting(const LoRaPacket_t *packet)
{
  /* Upstream WORs move from End Devices/farther repeaters toward a gateway.
   * If this node still has an upstream next hop, hold DATA until that next
   * part of the path is observed waking/forwarding the same WOR.
   */
  if ((packet->direction == PACKET_DIRECTION_UPSTREAM) &&
      (packet->txNodeType == PACKET_NODE_TYPE_END_DEVICE) &&
      ((packet->rxNodeID == 0U) || (packet->rxNodeID == nodeID)) &&
      (nextUptreamNodeID != nodeID))
  {
    return true;
  }

  if ((packet->direction == PACKET_DIRECTION_UPSTREAM) &&
      (packet->txNodeType == PACKET_NODE_TYPE_REPEATER) &&
      (nextUptreamNodeID != nodeID))
  {
    return true;
  }

  if ((packet->direction == PACKET_DIRECTION_DOWNSTREAM) &&
      ((packet->txNodeType == PACKET_NODE_TYPE_REPEATER) ||
       (packet->txNodeType == PACKET_NODE_TYPE_GATEWAY)) &&
      (nextDownstreamNodeID != nodeID))
  {
    return true;
  }

  return false;
}


static bool WorAckWait_ShouldClearWaiting(const LoRaPacket_t *packet)
{
  /* Only the same WOR packet, moving in the same direction, can clear the wait. */
  if ((!awaitingWorAck) ||
      (packet->packetID != awaitingWorPacketID) ||
      (packet->direction != awaitingWorDirection))
  {
    return false;
  }

  /* For upstream, the forthcoming node is closer to the gateway, so its
   * DistanceValue must be lower than this repeater's DistanceValue.
   */
  if ((packet->direction == PACKET_DIRECTION_UPSTREAM) &&
      ((packet->txNodeType == PACKET_NODE_TYPE_REPEATER) ||
       (packet->txNodeType == PACKET_NODE_TYPE_GATEWAY)) &&
      (packet->txDistanceValue < distanceValue))
  {
    return true;
  }

  /* For downstream, the forthcoming node is farther from the gateway, so its
   * DistanceValue must be higher than this repeater's DistanceValue.
   */
  if ((packet->direction == PACKET_DIRECTION_DOWNSTREAM) &&
      (packet->txNodeType == PACKET_NODE_TYPE_REPEATER) &&
      (packet->txDistanceValue > distanceValue))
  {
    return true;
  }

  return false;
}
