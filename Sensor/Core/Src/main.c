/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
uint8_t sensor_status = 0; // 用于存储传感器状态的变量
uint8_t prev_status = 0;   // 存储上一次的传感器状态
char uart_buffer[50];      // UART发送缓冲区
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 清除之前的状态
    sensor_status = 0;
    
    // 读取所有传感器状态并记录（低电平有效）
    if(HAL_GPIO_ReadPin(GPIOF, L1_Pin) == GPIO_PIN_RESET) 
      sensor_status |= 0x01;  // L1 传感器激活
      
    if(HAL_GPIO_ReadPin(GPIOA, L2_Pin) == GPIO_PIN_RESET) 
      sensor_status |= 0x02;  // L2 传感器激活
      
    if(HAL_GPIO_ReadPin(GPIOA, L3_Pin) == GPIO_PIN_RESET) 
      sensor_status |= 0x04;  // L3 传感器激活
      
    if(HAL_GPIO_ReadPin(GPIOF, L4_Pin) == GPIO_PIN_RESET) 
      sensor_status |= 0x08;  // L4 传感器激活
      
    if(HAL_GPIO_ReadPin(GPIOF, L5_Pin) == GPIO_PIN_RESET) 
      sensor_status |= 0x10;  // L5 传感器激活
      
    if(HAL_GPIO_ReadPin(GPIOG, L6_Pin) == GPIO_PIN_RESET) 
      sensor_status |= 0x20;  // L6 传感器激活
      
    if(HAL_GPIO_ReadPin(GPIOG, L7_Pin) == GPIO_PIN_RESET) 
      sensor_status |= 0x40;  // L7 传感器激活
    
    // 如果有传感器状态变化且有传感器被激活，发送数据
    if((sensor_status != prev_status) && (sensor_status > 0))
    {
      // 格式化消息
      sprintf(uart_buffer, "Sensors: %c%c%c%c%c%c%c\r\n", 
              (sensor_status & 0x01) ? '1' : '0',
              (sensor_status & 0x02) ? '1' : '0',
              (sensor_status & 0x04) ? '1' : '0',
              (sensor_status & 0x08) ? '1' : '0',
              (sensor_status & 0x10) ? '1' : '0',
              (sensor_status & 0x20) ? '1' : '0',
              (sensor_status & 0x40) ? '1' : '0');
      
      // 通过UART发送数据
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
    }
    
    // 更新LED状态 - L1传感器被激活时点亮LED
    if(sensor_status & 0x01){
      HAL_GPIO_WritePin(GPIOF, LED_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(GPIOF, LED_Pin, GPIO_PIN_RESET);
    }
    
    // 保存当前状态
    prev_status = sensor_status;
    
    // 短暂延时以减少串口发送频率
    HAL_Delay(50);
    
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
    if(HAL_GPIO_ReadPin(GPIOF, L1_Pin)){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
