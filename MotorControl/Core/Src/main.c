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
#include "mpu6050.h"
#include <string.h>
#include <math.h>
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
///陈which小女友hyf
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 64   //接收字符串长度A
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float duty[2];
int32_t current_total[2];
int32_t pre_total[2];
float rpm[2] = {0.0f};
float pre_rpm[2]={0.0f};
uint32_t last_key_time = 0;
double g_yaw=0.0f;

extern uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
extern void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *Data);
char uart_buffer[50];      // UART发送缓冲区
MPU6050_t MPU6050;         // MPU6050数据结构
uint8_t mode=0;
double origine_angle=0.0f;
double target_angle=40.0f;
double angle=0.0f;
int8_t circle=0;

volatile char rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;
volatile uint8_t rx_ready = 0;
char rx_char;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Send_MultiData_FireWater(float speed_rpm, float pidsetpoint,  float speed_rpm_,  float pidsetpoint_) {
    char buffer[64];
    int length = snprintf(buffer, sizeof(buffer), "%.2f, %.2f,%.2f,%.2f\r\n", speed_rpm, pidsetpoint, speed_rpm_, pidsetpoint_);

    // 通过 USART 发送字符串
    if (HAL_UART_Transmit(&huart1, (uint8_t*)buffer, length, 100) != HAL_OK) {
        Error_Handler();
    }
}


void CalculateYaw_Filtered(double gyro_z, double dt)
{
    // 静态变量保存yaw和bias
    static double yaw = 0.0;
    static double bias = 0.0;
    
    // 如果测量值很小，认为设备静止，可以更新零偏
    const double threshold = 2.5; // 阈值，根据实际情况调整
    if(fabs(gyro_z) < threshold)
    {
        bias = gyro_z;
    }
    
    // 用减去偏置后的角速度积分计算yaw
    double corrected_rate = gyro_z - bias;
    yaw += corrected_rate * dt;
    g_yaw=yaw*11;
    // 限制yaw范围在 [-180, 180]
    if (g_yaw > 180.0)
    {
        g_yaw -= 360.0;
    }
    else if (g_yaw < -180.0)
    {
        g_yaw += 360.0;
    }
    
    return;
}
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
    float max_output;    // 输出限幅
    float max_integral;  // 积分限幅
	  float prev_d;
} PID_Correct;
/* 全局变量 --------------------------------------------------------*/
PID_Controller left_motor_pid;     // PID控制器实例
PID_Controller right_motor_pid;     // PID控制器实例
PID_Correct correctl_pid;    //修正实例
PID_Correct correctr_pid;    //修正实例
float target_rpm[2];    // 目标转速 0为左轮B，1为右轮A
uint32_t pwm_max = 1000;       // PWM最大值对应100%占空比
float speed_time=0;//加速用时
float straight_error=0.0f;
float pre_straight_error=0.0f;
float correct[2];
uint16_t Direction[7]={0};
uint16_t Pre_Direction[7]={0};
float Kcl[3]={0.0f};//无线直行纠正系数
float Kcr[3]={0.0f};//无线直行纠正系数
void Kcl_init(float k0,float k1,float k2){
	Kcl[0]=k0;
	Kcl[1]=k1;
	Kcl[2]=k2;
}
void Kcr_init(float k0,float k1,float k2){
	Kcr[0]=k0;
	Kcr[1]=k1;
	Kcr[2]=k2;
}

//切换标志
uint8_t switch_count = 0; // 切换标志计数器

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
    pid->max_output = 200.0f;
    pid->max_integral = pid->max_output * 0.5f; // 积分限幅为输出的50%
	  pid->prev_d=0.0f;
}
/* correct计算（带抗饱和和滤波）-----------------------------------------*/
float PIDC_Compute(PID_Correct *pid, float rpm[],float pre_rpm[]) {
	  if((rpm[0]+pre_rpm[0]-2*target_rpm[0])<-2.0f||(rpm[0]+pre_rpm[0]-2*target_rpm[0])>2.0f)
			 return 0;
    //计算correct
		straight_error=(rpm[0]+pre_rpm[0])/2*pid->Ts-(rpm[1]+pre_rpm[1])/2*pid->Ts;
    
    // 比例项
    float P = pid->Kp * straight_error;
    
    // 积分项（带限幅）
    pid->integral += straight_error * pid->Ts;
    if(pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if(pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    float I = pid->Ki * pid->integral;
    
    // 微分项（带一阶低通滤波）
    float D = pid->Kd * (straight_error - pre_straight_error) / pid->Ts;
    D = 0.2f * D + 0.8f * pid->prev_d; // 低通滤波系数
    pid->prev_d = D;
    pre_straight_error = straight_error;
    
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
		PULSE_PER_REV[0]=1500;
		PULSE_PER_REV[1]=1500;
			 // 根据实际编码器参数修改
    const float SAMPLE_TIME = 0.1f;      // 定时中断周期（秒）
    
    rpm[0] = (delta_left / PULSE_PER_REV[0]) * (60.0f / SAMPLE_TIME);
		rpm[1] = (delta_right / PULSE_PER_REV[1]) * (60.0f / SAMPLE_TIME);
    OLED_ShowFloatNum(31,16,rpm[0],3,1,OLED_6X8);
		OLED_ShowFloatNum(31,24,rpm[1],3,1,OLED_6X8);
    // 更新上一次计数值
    pre_total[0] =current_total[0];
		pre_total[1] =current_total[1];
		
		rpm[0]=(rpm[0]>=0)?rpm[0]:-rpm[0];
		rpm[1]=(rpm[1]>=0)?rpm[1]:-rpm[1];
	  //读取灰度传感器
		for(int i=0;i<7;i++)
		  Pre_Direction[i]=Direction[i];
		if(HAL_GPIO_ReadPin(Direction_6_GPIO_Port,Direction_6_Pin)==GPIO_PIN_SET)//D_6
			Direction[6]=1;
		else
			Direction[6]=0;
		if(HAL_GPIO_ReadPin(Direction_5_GPIO_Port,Direction_5_Pin)==GPIO_PIN_SET)//5
			Direction[5]=1;
		else
			Direction[5]=0;
		if(HAL_GPIO_ReadPin(Direction_4_GPIO_Port,Direction_4_Pin)==GPIO_PIN_SET)//4
			Direction[4]=1;
		else
			Direction[4]=0;
		if(HAL_GPIO_ReadPin(Direction_3_GPIO_Port,Direction_3_Pin)==GPIO_PIN_SET)//3
			Direction[3]=1;
		else
			Direction[3]=0;
		if(HAL_GPIO_ReadPin(Direction_2_GPIO_Port,Direction_2_Pin)==GPIO_PIN_SET)//2
			Direction[2]=1;
		else
			Direction[2]=0;
		if(HAL_GPIO_ReadPin(Direction_1_GPIO_Port,Direction_1_Pin)==GPIO_PIN_SET)//1
			Direction[1]=1;
		else
			Direction[1]=0;
		if(HAL_GPIO_ReadPin(Direction_0_GPIO_Port,Direction_0_Pin)==GPIO_PIN_SET)//0
			Direction[0]=1;
		else
			Direction[0]=0;
	
		//mode
		switch (mode){
			case 0: { // 直行后停止
    // 直行 PID 控制
    correct[0] = PIDC_Compute(&correctl_pid, rpm, pre_rpm);
    correct[1] = PIDC_Compute(&correctr_pid, rpm, pre_rpm);

    // 检测切换标志
    uint8_t current_state = Direction[0] | Direction[1] | Direction[2] | Direction[3] | Direction[4] | Direction[5] | Direction[6];
    uint8_t previous_state = Pre_Direction[0] | Pre_Direction[1] | Pre_Direction[2] | Pre_Direction[3] | Pre_Direction[4] | Pre_Direction[5] | Pre_Direction[6];

    if ((current_state == 0 && previous_state != 0) || (current_state != 0 && previous_state == 0)) {
        // 检测到切换标志，停止
        target_rpm[0] = 0;
        target_rpm[1] = 0;
    }
    break;
}
			case 1: { // 正常循迹，第四个切换标志停止
    // 循迹 PID 控制
    correct[0] = PIDC_Compute(&correctl_pid, rpm, pre_rpm);
    correct[1] = PIDC_Compute(&correctr_pid, rpm, pre_rpm);

    // 检测切换标志
    uint8_t current_state = Direction[0] | Direction[1] | Direction[2] | Direction[3] | Direction[4] | Direction[5] | Direction[6];
    uint8_t previous_state = Pre_Direction[0] | Pre_Direction[1] | Pre_Direction[2] | Pre_Direction[3] | Pre_Direction[4] | Pre_Direction[5] | Pre_Direction[6];

    if ((current_state == 0 && previous_state != 0) || (current_state != 0 && previous_state == 0)) {
        switch_count++; // 记录切换标志
    }

    if (switch_count >= 4) {
        // 第四个切换标志，停止
        target_rpm[0] = 0;
        target_rpm[1] = 0;
    }
    break;
}
			case 2: { // 左转 45 度后直行，奇数切换标志循迹，偶数切换标志左转 45 度并直行，第四个切换标志停止
    // 检测切换标志
    uint8_t current_state = Direction[0] | Direction[1] | Direction[2] | Direction[3] | Direction[4] | Direction[5] | Direction[6];
    uint8_t previous_state = Pre_Direction[0] | Pre_Direction[1] | Pre_Direction[2] | Pre_Direction[3] | Pre_Direction[4] | Pre_Direction[5] | Pre_Direction[6];

    if ((current_state == 0 && previous_state != 0) || (current_state != 0 && previous_state == 0)) {
        switch_count++; // 记录切换标志
    }

    if (switch_count >= 4) {
        // 第四个切换标志，停止
        target_rpm[0] = 0;
        target_rpm[1] = 0;
    } else if (switch_count % 2 == 1) {
        // 奇数切换标志，循迹
        correct[0] = PIDC_Compute(&correctl_pid, rpm, pre_rpm);
        correct[1] = PIDC_Compute(&correctr_pid, rpm, pre_rpm);
    } else {
        // 偶数切换标志，左转 45 度
        double target_angle = angle + 45.0; // 目标角度
        while (fabs(angle - target_angle) > 1.0) {
            // 读取陀螺仪数据
            MPU6050_Read_All(&hi2c1, &MPU6050);
            CalculateYaw_Filtered(MPU6050.Gz, 0.01);
            angle = g_yaw;

            // 控制左转
            target_rpm[0] = -50; // 左轮减速
            target_rpm[1] = 50;  // 右轮加速
        }
        // 恢复直行
        target_rpm[0] = 100;
        target_rpm[1] = 100;
    }
    break;
}
			case 3: { // 同模式 3，但在第 16 个切换标志停止
    // 检测切换标志
    uint8_t current_state = Direction[0] | Direction[1] | Direction[2] | Direction[3] | Direction[4] | Direction[5] | Direction[6];
    uint8_t previous_state = Pre_Direction[0] | Pre_Direction[1] | Pre_Direction[2] | Pre_Direction[3] | Pre_Direction[4] | Pre_Direction[5] | Pre_Direction[6];

    if ((current_state == 0 && previous_state != 0) || (current_state != 0 && previous_state == 0)) {
        switch_count++; // 记录切换标志
    }

    if (switch_count >= 16) {
        // 第 16 个切换标志，停止
        target_rpm[0] = 0;
        target_rpm[1] = 0;
    } else if (switch_count % 2 == 1) {
        // 奇数切换标志，循迹
        correct[0] = PIDC_Compute(&correctl_pid, rpm, pre_rpm);
        correct[1] = PIDC_Compute(&correctr_pid, rpm, pre_rpm);
    } else {
        // 偶数切换标志，左转 45 度
        double target_angle = angle + 45.0; // 目标角度
        while (fabs(angle - target_angle) > 1.0) {
            // 读取陀螺仪数据
            MPU6050_Read_All(&hi2c1, &MPU6050);
            CalculateYaw_Filtered(MPU6050.Gz, 0.01);
            angle = g_yaw;

            // 控制左转
            target_rpm[0] = -50; // 左轮减速
            target_rpm[1] = 50;  // 右轮加速
        }
        // 恢复直行
        target_rpm[0] = 100;
        target_rpm[1] = 100;
    }
    break;
       }
			  }
		// PID计算
    float pwm_left = PID_Compute(&left_motor_pid, target_rpm[0]-correct[0], rpm[0]);
		float pwm_right = PID_Compute(&right_motor_pid, target_rpm[1]+correct[1], rpm[1]);
		// 更新PWM输出
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, (uint32_t)pwm_left);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, (uint32_t)pwm_right);
		duty[0]=pwm_left/10;
		OLED_ShowFloatNum(43,0,duty[0],2,4,OLED_6X8);
		duty[1]=pwm_right/10;
		OLED_ShowFloatNum(43,8,duty[1],2,4,OLED_6X8);
		pre_rpm[0]=rpm[0];
		pre_rpm[1]=rpm[1];
		OLED_ShowNum(37,48,mode,1,OLED_6X8);
		//Send_MultiData_FireWater(rpm[0],target_rpm[0],rpm[1],target_rpm[1]);
  }
	else if(htim->Instance==TIM3){
		current_total[0] +=(TIM3->CR1 & TIM_CR1_DIR) ? -65536 :65536;
	}
	else if(htim->Instance==TIM2){
		current_total[1] +=(TIM2->CR1 & TIM_CR1_DIR) ? -65536 : 65536;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (HAL_GetTick() - last_key_time > 200) { // 200ms防抖
        last_key_time = HAL_GetTick();
        correct[0] = 0;
        correct[1] = 0;
        straight_error = 0;
        pre_straight_error = 0;
        switch_count = 0; // 重置计数器
        if (GPIO_Pin == Key0_Pin) {
            mode = 0;
            circle = 0;
        } else if (GPIO_Pin == Key1_Pin) {
            mode = 1;
            circle = 0;
        } else if (GPIO_Pin == Key2_Pin) {
            mode = 2;
            circle = 0;
        } else if (GPIO_Pin == Key3_Pin) {
            mode = 3;
            circle = 0;
        }
    }
}
// 解析数据并更新PID参数
/*void Parse_Data() {
    char *token;
    float values[3];
    uint8_t count = 0;

    token = strtok((char *)rx_buffer, ",");
    while (token != NULL && count < 3) {
        values[count++] = atof(token);
        token = strtok(NULL, ",");
    }

    if (count == 3) { // 确保接收到三个值
        right_motor_pid.Kp = values[0];
        right_motor_pid.Ki = values[1];
        right_motor_pid.Kd = values[2];
    }
	}
*/
// USART接收中断回调函数
/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) { // 确保是目标USART
        // 检测结束符（回车或换行）
        if (rx_char == '\r' || rx_char == '\n') {
            if (rx_index > 0) { // 非空数据
                rx_buffer[rx_index] = '\0'; // 终止字符串
                rx_ready = 1; // 设置标志位
                rx_index = 0; // 重置索引
            }
        } else {
            // 将字符存入缓冲区，防止溢出
            if (rx_index < RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = rx_char;
            } else {
                // 缓冲区满，重置索引（可选：错误处理）
                rx_index = 0;
            }
        }
        // 重新启动接收中断
         HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_char, 1);
    }
}*/
	

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	// PID参数初始化（需根据实际系统调整）
  /*if(HAL_GPIO_ReadPin(BIN1_GPIO_Port,BIN1_Pin)==GPIO_PIN_SET){
    TIM2->CNT=65535;
  }*/
	target_rpm[0]=100.0f;
	target_rpm[1]=100.0f;
  PID_Init(&left_motor_pid, 6.0f, 0.6f, 0.1f, 0.1f); //Kp=0.6,Ki=0.5,Kd=0.1
	PID_Init(&right_motor_pid, 6.0f, 0.6f, 0.1f, 0.1f); // Kp=4.5,Ki=3.0;Kd=0.0
	PIDC_Init(&correctl_pid, 40.0f,20.0f,2.0f,0.1f);
	PIDC_Init(&correctr_pid, 40.0f,20.0f,2.0f,0.1f);
	Kcl_init(400.0f,200.0f,100.0f);
	Kcr_init(400.0f,200.0f,100.0f);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
	MPU6050_Init(&hi2c1);
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
	OLED_ShowString(1,32,"MAX:350rpm",OLED_6X8);
	OLED_ShowString(1,40,"Angle:",OLED_6X8);
	OLED_ShowString(1,48,"MODE:",OLED_6X8);
	
	//USART
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_char, 1);  //启动 USART 接收中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    // 读取MPU6050数据
    MPU6050_Read_All(&hi2c1, &MPU6050);
	  //计算当前角度
		CalculateYaw_Filtered(MPU6050.Gz, 0.01);
		angle=g_yaw;
		OLED_ShowFloatNum(43,40,angle,3,2,OLED_6X8);
		OLED_Update();
    // 格式化MPU6050数据并发送
    sprintf(uart_buffer, "Accel: %.2f, %.2f, %.2f; Gyro: %.2f, %.2f, %.2f\r\n", 
            MPU6050.Ax, MPU6050.Ay, MPU6050.Az, 
            MPU6050.Gx, MPU6050.Gy, MPU6050.Gz);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
    
    sprintf(uart_buffer, "Yaw: %.2f\r\n", angle);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);
    
    
    // 短暂延时以减少串口发送频率
    HAL_Delay(100);
    
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
