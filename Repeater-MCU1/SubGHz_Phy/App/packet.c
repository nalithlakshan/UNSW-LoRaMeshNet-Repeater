/*
 * packet.c
 *
 *  Created on: May 11, 2026
 *      Author: Nalith
 */

#include "packet.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

/**
  ******************************************************************************************************
  * Encoded Packet Format: (16 bytes header + N bytes payload)
  * 
  * |Bytes| Field               | Description
  * |-----|---------------------|----------------------------------------------------------------------
  * | 1   | txNodeID            | ID of the transmitting node
  * 
  * | 1   | flags               | Bit  0  : packetType  (0-WOR, 1-DATA)
  *                             | Bits 1-2: direction   (00-Broadcast, 01-Upstream, 10-Downstream)
  *                             | Bits 3-4: txNodeType  (00-Gateway, 01-End Device, 10-Repeater)
  *                             | Bits 5-6: rxNodeType  (00-Gateway, 01-End Device, 10-Repeater)
  *                             | Bit  7  : positionLearningMode (0-Normal Operation, 1-Routing Mode)
  * 
  * | 2   | txDistanceValue     | Distance value of the tx node
  * | 1   | txBatteryPercentage | Battery percentage of the tx node
  * | 1   | rxNodeID            | ID of the intended receiving node (0 if broadcast)
  * | 2   | rxDistanceValue     | Distance value of the rx node
  * | 1   | ackNodeID           | ID of the node Acknowledged if packet is a WOR (0 if no ACK)
  * | 1   | nearestGwID         | ID of the nearest gateway to the tx node
  * | 2   | packetID            | packet ID (1 byte Origin Device ID + 1 byte Sequence Number)
  * | 2   | preambleSize        | Size of the preamble in number of symbols
  * | 2   | payloadSize         | Size of the payload in bytes
  * | N   | payload             | Payload bytes (N = payloadSize)
  ******************************************************************************************************
  */


/** 
  * @brief Encodes a LoRa packet into a byte buffer
  * @param packet Pointer to the LoRa packet to encode
  * @param buffer Pointer to the buffer to store the encoded packet
  * @param bufferSize Size of the buffer
  * @return Size of the encoded packet, or 0 if encoding failed
 */  
uint16_t Packet_Encode(const LoRaPacket_t *packet, uint8_t *buffer, uint16_t bufferSize)
{
  uint16_t index = 0;
  uint32_t requiredSize;
  uint8_t flags = 0;

  // Basic validation of input parameters
  if ((packet == NULL) || 
      (buffer == NULL) ||
      (packet->payloadSize > LORA_PACKET_MAX_PAYLOAD_SIZE))
  {
    return 0;
  }

  // Check if buffer is large enough for header + payload
  requiredSize = (uint32_t)LORA_PACKET_HEADER_SIZE + packet->payloadSize;
  if ((uint32_t)bufferSize < requiredSize)
  {
    return 0;
  }

  // Construct flags byte from packetType, direction, txNodeType, rxNodeType, and positionLearningMode
  flags |= (uint8_t)((packet->packetType & 0x01) << 0U);
  flags |= (uint8_t)((packet->direction  & 0x03) << 1U);
  flags |= (uint8_t)((packet->txNodeType & 0x03) << 3U);
  flags |= (uint8_t)((packet->rxNodeType & 0x03) << 5U);
  flags |= (uint8_t)((packet->positionLearningMode & 0x01) << 7U);

  // Encode header fields into buffer
  buffer[index++] = packet->txNodeID;
  buffer[index++] = flags;
  buffer[index++] = (uint8_t)(packet->txDistanceValue >> 8);
  buffer[index++] = (uint8_t)packet->txDistanceValue;
  buffer[index++] = packet->txBatteryPercentage;
  buffer[index++] = packet->rxNodeID;
  buffer[index++] = (uint8_t)(packet->rxDistanceValue >> 8);
  buffer[index++] = (uint8_t)packet->rxDistanceValue;
  buffer[index++] = packet->ackNodeID;
  buffer[index++] = packet->nearestGwID;
  buffer[index++] = (uint8_t)(packet->packetID >> 8);
  buffer[index++] = (uint8_t)packet->packetID;
  buffer[index++] = (uint8_t)(packet->preambleSize >> 8);
  buffer[index++] = (uint8_t)packet->preambleSize;
  buffer[index++] = (uint8_t)(packet->payloadSize >> 8);
  buffer[index++] = (uint8_t)packet->payloadSize;

  // Copy payload into buffer if present
  if (packet->payloadSize > 0U)
  {
    memcpy(&buffer[index], packet->payload, packet->payloadSize);
    index = (uint16_t)(index + packet->payloadSize);
  }

  // Return total size of encoded packet (header + payload)
  return index; 
}


/** 
  * @brief Decodes a packet byte buffer into a LoRa packet structure
  * @param buffer Pointer to the buffer containing the encoded packet
  * @return The decoded LoRa packet structure
 */
LoRaPacket_t Packet_Decode(const uint8_t *buffer)
{
  uint16_t index = 0;
  LoRaPacket_t packet = {0};
  uint8_t flags;

  packet.txNodeID = buffer[index++];
  
  flags = buffer[index++];
  packet.packetType = (uint8_t)((flags >> 0U) & 0x01U);
  packet.direction = (uint8_t)((flags >> 1U) & 0x03U);
  packet.txNodeType = (uint8_t)((flags >> 3U) & 0x03U);
  packet.rxNodeType = (uint8_t)((flags >> 5U) & 0x03U);
  packet.positionLearningMode = (uint8_t)((flags >> 7U) & 0x01U);
  
  packet.txDistanceValue      = (uint16_t)buffer[index++] << 8 |buffer[index++];
  packet.txBatteryPercentage  = buffer[index++];
  packet.rxNodeID             = buffer[index++];
  packet.rxDistanceValue      = (uint16_t)buffer[index++] << 8 | buffer[index++];
  packet.ackNodeID            = buffer[index++];
  packet.nearestGwID          = buffer[index++];
  packet.packetID             = (uint16_t)buffer[index++] << 8 | buffer[index++];
  packet.preambleSize         = (uint16_t)buffer[index++] << 8 | buffer[index++];
  packet.payloadSize          = (uint16_t)buffer[index++] << 8 | buffer[index++];


  if (packet.payloadSize > 0U && packet.payloadSize <= LORA_PACKET_MAX_PAYLOAD_SIZE)
  {
    memcpy(packet.payload, &buffer[index], packet.payloadSize);
  }

  return packet;
}


/** 
  * @brief Converts a LoRa packet structure into a human-readable string format
  * @param packet Pointer to the LoRa packet to convert
  * @return Pointer to a static buffer containing the string representation of the packet
 */
const char *Packet_To_String(const LoRaPacket_t *packet)
{
  static char PacketStringBuffer[LORA_PACKET_STRING_BUFFER_SIZE];

  if (packet == NULL)
  {
    PacketStringBuffer[0] = '\0';
    return PacketStringBuffer;
  }

  uint8_t sequenceNumber = (uint8_t)packet->packetID;
  uint8_t originNodeID = (uint8_t)(packet->packetID >> 8);
  const char *packetTypeString = (packet->packetType == PACKET_TYPE_WOR) ? "WOR" : "DATA";
  char packetDirection = (packet->direction == PACKET_DIRECTION_UPSTREAM) ? 'U' : (packet->direction == PACKET_DIRECTION_DOWNSTREAM) ? 'D' : 'B';
  char txNodeTypeChar = (packet->txNodeType == PACKET_NODE_TYPE_END_DEVICE) ? 'E' : (packet->txNodeType == PACKET_NODE_TYPE_REPEATER) ? 'R' : 'G';
  char rxNodeTypeChar = (packet->rxNodeType == PACKET_NODE_TYPE_END_DEVICE) ? 'E' : (packet->rxNodeType == PACKET_NODE_TYPE_REPEATER) ? 'R' : 'G';
  uint16_t payloadSize = packet->payloadSize;
  if (payloadSize > LORA_PACKET_MAX_PAYLOAD_SIZE)
  {
    payloadSize = LORA_PACKET_MAX_PAYLOAD_SIZE;
  }

  snprintf(PacketStringBuffer,
           sizeof(PacketStringBuffer),
           "|SeqNr:%u/%u|%s-%c|M:%u|Tx:%c%u(dv=%u,bat=%u%%)|Rx:%c%u(dv=%u)|Ack:%u|Gw:%u|pr=%u|pl=%u|%.*s|",
           originNodeID,
           sequenceNumber,
           packetTypeString,
           packetDirection,
           packet->positionLearningMode,
           txNodeTypeChar,
           packet->txNodeID,
           packet->txDistanceValue,
           packet->txBatteryPercentage,
           rxNodeTypeChar,
           packet->rxNodeID,
           packet->rxDistanceValue,
           packet->ackNodeID,
           packet->nearestGwID,
           packet->preambleSize,
           packet->payloadSize,
           payloadSize,
           (const char *)packet->payload);

  return PacketStringBuffer;
}



