/*
 * transmitter.h
 *
 *  Created on: May 10, 2026
 *      Author: Nalith
 */

#ifndef APP_TRANSMITTER_H_
#define APP_TRANSMITTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "packet.h"

#include <stdbool.h>
#include <stdint.h>

#define TRANSMIT_BUFFER_MAX_PACKETS 10U

typedef struct
{
    LoRaPacket_t packets[TRANSMIT_BUFFER_MAX_PACKETS];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} TransmitBuffer_t;

extern TransmitBuffer_t Transmit_Buffer;
extern volatile bool edTxCycleActive;

bool Transmitter_Submit(const LoRaPacket_t *packet);
void Transmitter_TxLoop(void);
void Transmitter_Init(void);
void Transmitter_StartPeriodicED(void);
void Transmitter_OnTxDone(void);
void Transmitter_OnTxTimeout(void);
void Transmitter_OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void Transmitter_OnRxTimeout(void);
void Transmitter_OnRxError(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TRANSMITTER_H_ */
