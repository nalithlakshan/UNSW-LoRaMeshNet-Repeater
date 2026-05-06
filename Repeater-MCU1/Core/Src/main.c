/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "app_subghz_phy.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_timer.h"
#include <stdio.h>
#include <string.h>
#include "sys_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart1_rx_dma_buffer [1024];
uint8_t uart2_rx_dma_buffer [1024];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SubGHz_Phy_Init();
  MX_USART1_UART_Init();
  // MX_RTC_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1); // Turn on LED2 to indicate that the system is running
  
  HAL_GPIO_WritePin(LOAD_SWITCH_GPIO_Port, LOAD_SWITCH_Pin, 1);

  // HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) &uart1_rx_dma_buffer, sizeof(uart1_rx_dma_buffer));
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) &uart2_rx_dma_buffer, sizeof(uart2_rx_dma_buffer));
  // __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  // __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  while(1){
//	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
//	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
//	  char msg[] = "This is an example only!!!\r\n";
//	  HAL_UART_Transmit(&huart1, (uint8_t *)msg, sizeof(msg) - 1, HAL_MAX_DELAY);
//	  HAL_Delay(1000);
//	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
//	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
//	  HAL_Delay(1000);
//	  }
    /* USER CODE END WHILE */
    MX_SubGHz_Phy_Process();

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
// {
//   if (huart->Instance == USART2) 
//   {
//     APP_LOG(TS_OFF, VLEVEL_M, "Response from ESP-01: %s\r\n", uart2_rx_dma_buffer);
//     HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) &uart2_rx_dma_buffer, sizeof(uart2_rx_dma_buffer));
//     __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
//   }

//   if (huart->Instance == USART1)
//   {
//     APP_LOG(TS_OFF, VLEVEL_M, "Message Sent to ESP-01: %s\r\n", uart1_rx_dma_buffer);
//     HAL_UART_Transmit(&huart2, (uint8_t *)uart1_rx_dma_buffer, strlen(uart1_rx_dma_buffer), HAL_MAX_DELAY);
//     HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) &uart1_rx_dma_buffer, sizeof(uart1_rx_dma_buffer));
//     __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

//   }
// }

// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
// {
//   if (huart->Instance == USART1)
//   {
//     // Handle UART error (e.g., log it, reset the UART, etc.)
//     // For this example, we'll just toggle an LED to indicate an error
//     HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//   }
// }

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
