/*
 * i2c_pkt_transfer.c
 *
 *  Created on: Jun 2, 2026
 *      Author: Nalith
 */

#include "i2c_pkt_transfer.h"

#include "i2c.h"
#include "main.h"
#include "stm32_timer.h"
#include "subghz_phy_app.h"
#include "sys_app.h"
#include <stdlib.h>

#define MCU1_I2C_ADDRESS_7BIT        0x10
#define I2C_BUSY_RETRY_TIMEOUT_MS    2000
#define I2C_BUSY_RETRY_MAX_DELAY_MS  20
#define I2C_TX_TIMEOUT_MS            100


HAL_StatusTypeDef WakeMCU1andTransferData(uint8_t *data)
{
  HAL_StatusTypeDef status;
  uint32_t retryStartTick;
  uint32_t retryDelay;

  if (data == NULL)
  {
    return HAL_ERROR;
  }

  // Wake up MCU1
  HAL_GPIO_WritePin(WAKE_MCU1_GPIO_Port, WAKE_MCU1_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  // Attempt to transfer data with retry on arbitration loss
  retryStartTick = HAL_GetTick();
  while ((HAL_GetTick() - retryStartTick) < I2C_BUSY_RETRY_TIMEOUT_MS)
  {
    status = HAL_I2C_Master_Transmit(&hi2c2,
                                     (uint16_t)(MCU1_I2C_ADDRESS_7BIT << 1),
                                     data,
                                     MAX_APP_BUFFER_SIZE,
                                     I2C_TX_TIMEOUT_MS);

    if (status == HAL_OK)
    {
      APP_LOG(TS_OFF, VLEVEL_M, "Data transferred to MCU1\r\n");
      HAL_Delay(5);
      HAL_GPIO_WritePin(WAKE_MCU1_GPIO_Port, WAKE_MCU1_Pin, GPIO_PIN_RESET);
      return HAL_OK;
    }

    uint32_t err = HAL_I2C_GetError(&hi2c2);
    if (err & HAL_I2C_ERROR_ARLO)
    {
      // This master lost arbitration. Back off and retry later
      retryDelay = (rand() % I2C_BUSY_RETRY_MAX_DELAY_MS) + 1;
      APP_LOG(TS_OFF, VLEVEL_M, "I2C2 arbitration lost, will retry in %d ms\r\n", (int)retryDelay);
      HAL_Delay(retryDelay);
      continue;
    }

    // For other errors, break the loop and handle the error
    APP_LOG(TS_OFF, VLEVEL_M, "I2C ERROR (status=%d, error=%u)\r\n", (int)status, err);
    break;
  }

  // Failed to transfer data after retries, reset wake pin and log error
  HAL_GPIO_WritePin(WAKE_MCU1_GPIO_Port, WAKE_MCU1_Pin, GPIO_PIN_RESET);
  APP_LOG(TS_OFF, VLEVEL_M, "Failed to transfer data to MCU1\r\n");
  if ((HAL_GetTick() - retryStartTick) >= I2C_BUSY_RETRY_TIMEOUT_MS)
  {
    status = HAL_TIMEOUT;
  }
  return status;
}
