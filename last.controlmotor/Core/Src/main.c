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
uint8_t out_buffer[5];
float duty[2];
int32_t current_total[2];
int32_t pre_total[2];
float rpm[2] = {0.0f};
float pre_rpm[2]={0.0f};
uint32_t last_key_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float Ts;           // 采样时间(s)
    float integral;      // 积分项
    float prev_error;    // 上次误差
    float max_output;    // 输出限幅
    float max_integral;  // 积分限幅
	  float prev_d;
} PID_Controller;
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float Ts;           // 采样时间(s)
    float integral;      // 积分项
    float prev_straight_error;    // 上次误差
    float max_output;    // 输出限幅
    float max_integral;  // 积分限幅
	  float prev_d;
} PID_Correct;
/* 全局变量 --------------------------------------------------------*/
PID_Controller left_motor_pid;     // PID控制器实例
PID_Controller right_motor_pid;     // PID控制器实例
PID_Correct correct_pid;    //修正实例
float target_rpm[2];    // 目标转速 0为左轮B，1为右轮A
uint32_t pwm_max = 1000;       // PWM最大值对应100%占空比
float speed_time=0;//加速用时
float straight_error=0.0f;
float pre_straight_error=0.0f;
uint32_t correct;
float Kcp,Kci;
/* PID初始化 --------------------------------------------------------*/
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float Ts) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Ts = Ts;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->max_output = (float)pwm_max;
    pid->max_integral = pid->max_output * 0.5f; // 积分限幅为输出的50%
	  pid->prev_d=0.0f;
}
/* PID计算（带抗饱和和滤波）-----------------------------------------*/
float PID_Compute(PID_Controller *pid, float setpoint, float measurement) {
    // 计算误差
    float error = setpoint - measurement;
    
    // 比例项
    float P = pid->Kp * error;
    
    // 积分项（带限幅）
    pid->integral += error * pid->Ts;
    if(pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if(pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    float I = pid->Ki * pid->integral;
    
    // 微分项（带一阶低通滤波）
    float D = pid->Kd * (error - pid->prev_error) / pid->Ts;
    D = 0.2f * D + 0.8f * pid->prev_d; // 低通滤波系数
    pid->prev_d = D;
    pid->prev_error = error;
    
    // 计算输出
    float output = P + I + D;
    
    // 输出限幅
    if(output > pid->max_output) output = pid->max_output;
    else if(output < 0.0f) output = 0.0f;
    
    return output;
}
/* correct初始化 --------------------------------------------------------*/
void PIDC_Init(PID_Correct *pid, float Kp, float Ki, float Kd, float Ts) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Ts = Ts;
    pid->integral = 0.0f;
    pid->prev_straight_error = 0.0f;
    pid->max_output = 200.0f;
    pid->max_integral = pid->max_output * 0.5f; // 积分限幅为输出的50%
	  pid->prev_d=0.0f;
}
/* correct计算（带抗饱和和滤波）-----------------------------------------*/
float PIDC_Compute(PID_Correct *pid, float rpm[],float pre_rpm[]) {
    //计算correct
		straight_error+=((rpm[0]+pre_rpm[0])/2*pid->Ts-(rpm[1]+pre_rpm[1])/2*pid->Ts);
    
    // 比例项
    float P = pid->Kp * straight_error;
    
    // 积分项（带限幅）
    pid->integral += straight_error * pid->Ts;
    if(pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if(pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    float I = pid->Ki * pid->integral;
    
    // 微分项（带一阶低通滤波）
    float D = pid->Kd * (straight_error - pid->prev_straight_error) / pid->Ts;
    D = 0.2f * D + 0.8f * pid->prev_d; // 低通滤波系数
    pid->prev_d = D;
    pid->prev_straight_error = straight_error;
    
    // 计算输出
    float output = P + I + D;
    
    // 输出限幅
    if(output > pid->max_output) output = pid->max_output;
    else if(output < 0.0f) output = 0.0f;
    
    return output;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    // 读取当前编码器计数值
		if((TIM3->CR1 & TIM_CR1_DIR)&&(TIM2->CR1 & TIM_CR1_DIR)){
			current_total[0]-=(65535-TIM3->CNT);
		  TIM3->CNT=65535;
			current_total[1]-=(65535-TIM2->CNT);
		  TIM2->CNT=65535;
		}
		else{
		  current_total[0]+=TIM3->CNT;
			TIM3->CNT=0;
			current_total[1]+=TIM2->CNT;
			TIM2->CNT=0;
		}
    // 计算增量（处理溢出）
    int32_t delta_left = current_total[0]-pre_total[0];
		int32_t delta_right = current_total[1]-pre_total[1];
    // 计算转速（单位：RPM）
    // 假设编码器为4线正交编码，每转产生N个脉冲
    float PULSE_PER_REV[2];
		PULSE_PER_REV[0]=500;
		PULSE_PER_REV[1]=1500;
			 // 根据实际编码器参数修改
    const float SAMPLE_TIME = 0.1f;      // 定时中断周期（秒）
    
    rpm[0] = (delta_left / PULSE_PER_REV[0]) * (60.0f / SAMPLE_TIME);
		rpm[1] = (delta_right / PULSE_PER_REV[1]) * (60.0f / SAMPLE_TIME);
		rpm[0]=-rpm[0];
		rpm[1]=-rpm[1];
    OLED_ShowFloatNum(31,16,rpm[0],3,1,OLED_6X8);
		OLED_ShowFloatNum(31,24,rpm[1],3,1,OLED_6X8);
    // 更新上一次计数值
    pre_total[0] =current_total[0];
		pre_total[1] =current_total[1];
		
		rpm[0]=(rpm[0]>=0)?rpm[0]:-rpm[0];
		rpm[1]=(rpm[1]>=0)?rpm[1]:-rpm[1];
		//correct计算
		correct=PIDC_Compute(&correct_pid,rpm,pre_rpm);
		// PID计算
    float pwm_left = PID_Compute(&left_motor_pid, target_rpm[0], rpm[0]);
		float pwm_right = PID_Compute(&right_motor_pid, target_rpm[1], rpm[1]);
		// 更新PWM输出
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, (uint32_t)pwm_left);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, (uint32_t)pwm_right);
		duty[0]=pwm_left/10;
		OLED_ShowFloatNum(43,0,duty[0],2,4,OLED_6X8);
		duty[1]=pwm_right/10;
		OLED_ShowFloatNum(43,8,duty[1],2,4,OLED_6X8);
		pre_rpm[0]=rpm[0];
		pre_rpm[1]=rpm[1];
		OLED_ShowFloatNum(67,40,speed_time,1,1,OLED_6X8);
		OLED_Update();
  }
	else if(htim->Instance==TIM3){
		current_total[0] +=(TIM3->CR1 & TIM_CR1_DIR) ? -65536 :65536;
	}
	else if(htim->Instance==TIM2){
		current_total[1] +=(TIM2->CR1 & TIM_CR1_DIR) ? -65536 : 65536;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
if(HAL_GetTick() - last_key_time > 200) { // 200ms防抖
    last_key_time = HAL_GetTick();
	if(GPIO_Pin==Key1_Pin){
		HAL_GPIO_TogglePin(BIN2_GPIO_Port,BIN2_Pin);
		HAL_GPIO_TogglePin(BIN1_GPIO_Port,BIN1_Pin);
		HAL_GPIO_TogglePin(bin1_GPIO_Port,bin1_Pin);
		HAL_GPIO_TogglePin(bin2_GPIO_Port,bin2_Pin);
		left_motor_pid.Kp=(HAL_GPIO_ReadPin(BIN1_GPIO_Port,BIN1_Pin)==GPIO_PIN_SET)? 0.069 : 0.072f;
		right_motor_pid.Kp=(HAL_GPIO_ReadPin(BIN1_GPIO_Port,bin2_Pin)==GPIO_PIN_SET)? 0.295f : 0.3f;
		speed_time=0.0f;
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
  MX_TIM11_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	// PID参数初始化（需根据实际系统调整）
	target_rpm[0]=2000.0f;
	target_rpm[1]=2000.0f;
  PID_Init(&left_motor_pid, 0.0f, 0.5f, 0.0f, 0.1f); // Kp=0.069, Ki=0.01, Kd=0.04,Ts=0.1;
	PID_Init(&right_motor_pid, 0.0f, 0.5f, 0.0f, 0.1f); // Kp=0.295, Ki=0.01, Kd=0.01,Ts=0.1;
	PIDC_Init(&correct_pid, 0.0f,0.5f,0.0f,0.1f);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
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
	OLED_ShowString(1,32,"MAX:170rpm",OLED_6X8);
	OLED_ShowString(1,40,"Speed Time:",OLED_6X8);
	OLED_Update();
	//USART
  HAL_UART_Receive_IT(&huart1, rxBuffer,RX_CMD_LEN);  //中断方式接收5个字节
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 100);
		HAL_Delay(10);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 100);
		HAL_Delay(10);*/
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
