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
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "stdio.h"
#include <string.h>
#include "stdlib.h"
#include "motor_control.h"
#include "tramsmit.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
///陈which小女友hyf
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t last_key_time = 0;



float x=0.0;
float y=0.0;
float wheel_radius=3.2;    //单位厘米
float linear;
float wheel_distance=19.5;

float ground_x=80.0;
float ground_y=100.0;
float angle_to_hudu=3.1415926/180.0;

float x_direction=0.0;
float x_delt=0.0;
float y_direction=0.0;
float y_delt=0.0;





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




   //计算小车x轴和y轴位移

void Distance_x_y(){
	
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {  //T=0.01s
    
		//计算角度
		  // 读取MPU6050数据
	  //计算当前角度
		//CalculateYaw_Filtered(MPU6050.Gz  ,SAMPLE_TIME);
		//read_Direction_flag(Direction,Pre_Direction,7);
    motor_control_in_TIM1();
	}
	else if(htim->Instance==TIM2){
		current_total[0] +=(TIM2->CR1 & TIM_CR1_DIR) ? -65536 :65536;
	}
	else if(htim->Instance==TIM3){
		current_total[1] +=(TIM3->CR1 & TIM_CR1_DIR) ? -65536 : 65536;
  }
	else if(htim->Instance==TIM4){  //T=0.1s
		OLED_ShowFloatNum(31,16,rpm[0],3,1,OLED_6X8);
		OLED_ShowFloatNum(31,24,rpm[1],3,1,OLED_6X8);
		//MPU6050_Read_All(&hi2c1, &MPU6050);
		OLED_ShowChar(37,32,k210_reserved_state,OLED_6X8);
		OLED_ShowChar(91,32,openmv_state,OLED_6X8);
		duty[0]=pwm_left/10;
		OLED_ShowFloatNum(43,0,duty[0],2,4,OLED_6X8);
		duty[1]=pwm_right/10;
		OLED_ShowFloatNum(43,8,duty[1],2,4,OLED_6X8);
		OLED_ShowNum(37,40,mode,1,OLED_6X8);
		OLED_ShowNum(37,48,turning_flag,1,OLED_6X8);
		OLED_ShowFloatNum(49,48,get_distance(),3,1,OLED_6X8);
		OLED_ShowNum(55,40,switch_count,1,OLED_6X8);
		OLED_Update();
		if(alarm_enable==1){
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 500);
			beep_time++;
		}
		if(beep_time>=5){
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
			alarm_enable=0;
			beep_time=0;
		}
		/*if (rx_ready) {
            Parse_Data();
            rx_ready = 0;
        }
		Send_MultiData_FireWater(rpm[0],target_rpm[0],rpm[1],target_rpm[1]);*/
		
     //格式化MPU6050数据并发送
    /*sprintf(uart_buffer, "Accel: %.2f, %.2f, %.2f; Gyro: %.2f, %.2f, %.2f\r\n", 
            MPU6050.Ax, MPU6050.Ay, MPU6050.Az, 
            MPU6050.Gx, MPU6050.Gy, MPU6050.Gz);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
    
    sprintf(uart_buffer, "Yaw: %.2f\r\n", angle);*/
    //HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
		/*__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 100);
		HAL_Delay(10);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 100);
		HAL_Delay(10);*/
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (HAL_GetTick() - last_key_time > 10) { // 200ms防抖
        last_key_time = HAL_GetTick();
        correct[0] = 0;
        correct[1] = 0;
        if (GPIO_Pin == Key1_Pin) {
            mode = 1;
					  load=0;
					  origin_count=current_total[0];
					  alarm_enable=1;
        } else if (GPIO_Pin == Key2_Pin) {
            mode = 2;
					  load=0;
					  origin_count=current_total[0];
					  alarm_enable=1;
        } else if (GPIO_Pin == Key3_Pin) {
            mode = 3;
					  load=0;
					  origin_count=current_total[0];
					  alarm_enable=1;
				}
				else if(GPIO_Pin==Key4_Pin){
					unload=0;
					stop_flag=0;
					alarm_enable=1;
				}
				else if(GPIO_Pin==Key5_Pin){
					load=0;
					stop_flag=0;
					alarm_enable=1;
				}
    }
}

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
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	MPU6050_Init(&hi2c1);
	
  /*if(HAL_GPIO_ReadPin(BIN1_GPIO_Port,BIN1_Pin)==GPIO_PIN_SET){
    TIM2->CNT=65535;
  }*/
	target_rpm[0]=100.0f;
	target_rpm[1]=100.0f;
  PID_Init(&left_motor_pid, 7.0f, 22.0f, 0.01f, SAMPLE_TIME,pwm_max); //    6.0f, 0.6f, 0.1f, SAMPLE_TIME
	PID_Init(&right_motor_pid, 7.0f, 22.0, 0.01f, SAMPLE_TIME,pwm_max); // 
	
	PID_Init(&angle_pid,6.0f,0.6f,0.01f,SAMPLE_TIME,target_rpm[0]);   //角度pid
	
	linear=wheel_radius*2*3.1415926/60;
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
	//OLED
	
	OLED_Init();
	OLED_ShowString(1,0,"LDuty:",OLED_6X8);
	OLED_ShowChar(67,0,'%',OLED_6X8);
	OLED_ShowString(1,8,"RDuty:",OLED_6X8);
	OLED_ShowChar(67,8,'%',OLED_6X8);
	OLED_ShowString(1,16,"LRPM:",OLED_6X8);
	OLED_ShowString(1,24,"RRPM:",OLED_6X8);
	OLED_ShowFloatNum(31,16,rpm[0],3,1,OLED_6X8);
	OLED_ShowFloatNum(31,24,rpm[1],3,1,OLED_6X8);
	OLED_ShowString(1,32,"K210:",OLED_6X8);
	OLED_ShowString(49,32,"Openm:",OLED_6X8);
	OLED_ShowString(1,40,"MODE:",OLED_6X8);
	OLED_ShowString(1,48,"Turn:",OLED_6X8);
	
	OLED_Update();
	//USART
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_char1, 1);  //启动 USART 接收中断
	HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_char3, 1);  //启动 USART 接收中断
	HAL_UART_Receive_IT(&huart6, (uint8_t*)&received_byte, 1);  //启动 USART 接收中断
	TIM2->CNT=65535;
	origin_count=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
