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
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "servo_protocol.h"
#include "laser_tracker.h"
#include <stdio.h>
#include <stdlib.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
char current_coord[BUF_SIZE]= {"xiangwan stupid donkey"};
int x1, y1, x2, y2, x3, y3, x4, y4, a, b, c, d;
  
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void parseCoordinates(const char *str, int *x1, int *y1, int *x2, int *y2, 
                      int *x3, int *y3, int *x4, int *y4, int *a, int *b, int *c, int *d) {
    const char *ptr = str;
    
    // 解析前四个顶点坐标（x1,y1;x2,y2;x3,y3;x4,y4）
    sscanf(ptr, "%d,%d;", x1, y1);
    ptr = strchr(ptr, ';') + 1;
    
    sscanf(ptr, "%d,%d;", x2, y2);
    ptr = strchr(ptr, ';') + 1;
    
    sscanf(ptr, "%d,%d;", x3, y3);
    ptr = strchr(ptr, ';') + 1;
    
    sscanf(ptr, "%d,%d;", x4, y4);
    ptr = strchr(ptr, ';') + 1;
    
    // 解析激光红点（a,b）和绿点坐标（c,d）
    sscanf(ptr, "%d,%d;", a, b); 
    ptr = strchr(ptr, ';') + 1;
    
    sscanf(ptr, "%d,%d", c, d);  
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int test1 = 6;
uint32_t last_key_time = 0;
uint32_t beeptime = 0;
int circle1 = 0 ;
int circle2 = 0 ;
int circle3 = 0 ;
//按键控制
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (HAL_GetTick() - last_key_time > 10) { // 200ms防抖
        last_key_time = HAL_GetTick();
        
        if (GPIO_Pin == Key1_Pin) {
					//回到中心点
            Servo_SetPosition(1, 2865, 500);Servo_SetPosition(2, 3500, 500);
        }
				else if (GPIO_Pin == Key2_Pin) {
            circle1 = 1 ;
        } 
				else if (GPIO_Pin == Key3_Pin) {
            circle2 = 1 ;
				}
				else if(GPIO_Pin==Key4_Pin){
					  circle3 = 1 ;
				}
				
    }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {  
		
				// 4. 显示更新
        OLED_Clear();
				OLED_ShowString(4, 10, "dj1:", OLED_6X8);
	      OLED_ShowString(1,32,"Pos:",OLED_6X8);
				OLED_ShowString(30,32,current_coord,OLED_6X8);
			  OLED_ShowNum(30, 10, a, 3 , OLED_6X8);
		    OLED_ShowNum(60, 10, b, 3 , OLED_6X8);
				
        OLED_ShowString(4, 50, "any:", OLED_6X8);
        OLED_ShowSignedNum(40, 50, c, 4, OLED_6X8);
        OLED_ShowString(72, 50, ",", OLED_6X8);
        OLED_ShowSignedNum(78, 50, d, 4, OLED_6X8);
        OLED_Update();
					
		
		
		
		
	}
}


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
  MX_USART6_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
	
	OLED_Init();
	
	Servo_Init(&huart6);
	  
	// 标记是否为首次初始化
  uint8_t first_init = 1;
   
    
		
    LaserTracker tracker = {0};
//    // 在main()初始化部分添加默认顶点（测试用）
//    Point default_vertices[4] = {{200, 200}, {400, 200}, {400, 300}, {200, 300}};
//    init_tracker(&tracker, default_vertices);
//    generate_path(&tracker, 20); // 强制生成路径，用于测试
//		test1 = tracker.current_target;
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(circle1){
		Servo_SetPosition(0x01, 2770, 300);Servo_SetPosition(0x02, 3420, 300);
		HAL_Delay(1000);
		Servo_SetPosition(0x01, 2955, 300);Servo_SetPosition(0x02, 3417, 300);
		HAL_Delay(800);	
    Servo_SetPosition(0x01, 2950, 300);Servo_SetPosition(0x02, 3587, 300);
		HAL_Delay(800);
		Servo_SetPosition(0x01, 2770, 300);Servo_SetPosition(0x02, 3587, 300);
		HAL_Delay(800);
		Servo_SetPosition(0x01, 2770, 300);Servo_SetPosition(0x02, 3420, 300);
			
			circle1 = 0 ;
    }    
        
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
    // 1. 更新坐标数据
        if (coord_updated) {
            strncpy(current_coord, coord, BUF_SIZE);
            coord_updated = 0;
            
            // 解析新坐标
            parseCoordinates(current_coord, &x1, &y1, &x2, &y2, &x3, &y3, &x4, &y4, &a, &b, &c, &d);
            
            // 仅在首次初始化时生成路径
        
      }
				
		// 2. 更新激光位置
        Point cam_pos = {a, b};
        update_laser_position(&tracker, cam_pos);
		// 3. 执行追踪控制
        if (circle2) {
					if(first_init) {
            Point vertices[4] = {{x1,y1}, {x2,y2}, {x3,y3}, {x4,y4}};
						init_tracker(&tracker, vertices);
            generate_path(&tracker, 25);
            tracker.current_target = 0;
            first_init = 0; // 清除首次初始化标记
        }
            control_laser(&tracker);
				    HAL_Delay(55);
        }
				
				 if(circle3){
				    control_green_light(a,b,c,d);
         }   
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
