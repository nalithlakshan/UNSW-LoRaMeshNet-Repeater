/*
 * packet.h
 *
 *  Created on: May 11, 2026
 *      Author: Nalith
 */

#ifndef APP_PACKET_H_
#define APP_PACKET_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  PACKET_NODE_TYPE_GATEWAY    = 0x00,
  PACKET_NODE_TYPE_END_DEVICE = 0x01,
  PACKET_NODE_TYPE_REPEATER   = 0x02
} PacketNodeType_t;

typedef enum
{
  PACKET_TYPE_WOR  = 0x00,
  PACKET_TYPE_DATA = 0x01
} PacketType_t;

typedef enum
{
  PACKET_DIRECTION_BROADCAST  = 0x00,
  PACKET_DIRECTION_UPSTREAM   = 0x01,
  PACKET_DIRECTION_DOWNSTREAM = 0x02
} PacketDirection_t;

#define LORA_PACKET_DEFAULT_RX_NODE_ID              0U
#define LORA_PACKET_HEADER_SIZE                     16U
#define LORA_PACKET_MAX_PAYLOAD_SIZE                128U
#define LORA_PACKET_STRING_BUFFER_SIZE              384U

typedef struct
{
  uint8_t txNodeID;
  uint8_t txNodeType : 2;
  uint16_t txDistanceValue;
  uint8_t txBatteryPercentage;
  uint8_t rxNodeID;
  uint8_t rxNodeType : 2;
  uint16_t rxDistanceValue;
  uint8_t ackNodeID;
  uint8_t nearestGwID;
  uint16_t packetID;
  uint8_t packetType : 1;
  uint8_t direction : 2;
  uint8_t positionLearningMode : 1;
  uint16_t preambleSize;
  uint16_t payloadSize;
  uint8_t payload[LORA_PACKET_MAX_PAYLOAD_SIZE];
} LoRaPacket_t;

uint16_t Packet_Encode(const LoRaPacket_t *packet, uint8_t *buffer, uint16_t bufferSize);
LoRaPacket_t Packet_Decode(const uint8_t *buffer);
const char *Packet_To_String(const LoRaPacket_t *packet);

#ifdef __cplusplus
}
#endif

#endif /* APP_PACKET_H_ */
