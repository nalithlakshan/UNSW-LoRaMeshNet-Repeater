/*
 * i2c_pkt_transfer.h
 *
 *  Created on: Jun 2, 2026
 *      Author: Nalith
 */

#ifndef APP_I2C_PKT_TRANSFER_H_
#define APP_I2C_PKT_TRANSFER_H_

#include "stm32wlxx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef WakeMCU1andTransferData(uint8_t *data);

#endif /* APP_I2C_PKT_TRANSFER_H_ */
