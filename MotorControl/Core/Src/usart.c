/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "rtc.h" 
extern float target_rpm[2];
uint8_t  proBuffer[10]="#S45;";  //用于处理的数据 
uint8_t  rxBuffer[10]="#H12;"; //接收数据缓冲区 
uint8_t  rxCompleted=RESET;   //HAL_UART_Receive_IT()接收是否完成 
uint8_t  isUploadTime=0;         //控制RTC周期唤醒中断里是否上传时间数据
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
extern PID_Controller left_motor_pid;     // PID控制器实例
extern PID_Controller right_motor_pid;     // PID控制器实例
extern float speed_time;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void updateRTCTime() 
{ 
  if (proBuffer[0] != '#') 
    return; 
  uint8_t  timeSection=proBuffer[1]; //类型字符 
  uint8_t  tmp10=proBuffer[2]-0x30; //十位数 
  uint8_t  tmp1 =proBuffer[3]-0x30; //个位数 
  uint8_t  val=10*tmp10+tmp1; 
  if (timeSection=='U') 
  { 
    isUploadTime=val; 
    return; 
  }  
//。。。。。。。。。。//中间代码略，见源程序 
  //HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);  //设置RTC时间 
}
void  on_UART_IDLE(UART_HandleTypeDef *huart)  
{ 
  if(__HAL_UART_GET_IT_SOURCE(huart,UART_IT_IDLE) == RESET)  
    return; 
  __HAL_UART_CLEAR_IDLEFLAG(huart);   //清除IDLE标志 
  __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);  //禁止IDLE中断 
  if (rxCompleted)  //接收到了一条指令 
  { 
    HAL_UART_Transmit_IT(huart,proBuffer,RX_CMD_LEN);  //指令字符串传回PC
    updateRTCTime(); 
    rxCompleted=RESET; 
    HAL_UART_Receive_IT(huart, rxBuffer,RX_CMD_LEN);  //再次启动接收 
  } 
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //串口接收完成回调函数 
{ 
  if (huart->Instance == USART1) 
  { 
    rxCompleted=SET;  //接收完成
		if(rxBuffer[0]=='r'&&rxBuffer[1]=='l'){
		  target_rpm[0]=0;
		  target_rpm[0]+=((rxBuffer[2]-'0')*1000+(rxBuffer[3]-'0')*100+(rxBuffer[4]-'0')*10);
			speed_time=0.0f;
		}
		else if(rxBuffer[0]=='r'&&rxBuffer[1]=='r'){
		  target_rpm[1]=0;
		  target_rpm[1]+=((rxBuffer[2]-'0')*1000+(rxBuffer[3]-'0')*100+(rxBuffer[4]-'0')*10);
			speed_time=0.0f;
		}
		else if(rxBuffer[0]=='p'){
			if(rxBuffer[1]=='l')
			  left_motor_pid.Kp=rxBuffer[2]-'0'+0.1*(rxBuffer[3]-'0')+0.01*(rxBuffer[4]-'0')+0.001*(rxBuffer[5]-'0');
			else
				right_motor_pid.Kp=rxBuffer[2]-'0'+0.1*(rxBuffer[3]-'0')+0.01*(rxBuffer[4]-'0')+0.001*(rxBuffer[5]-'0');
		}
		else if(rxBuffer[0]=='i'){
			if(rxBuffer[1]=='l')
			  left_motor_pid.Ki=rxBuffer[2]-'0'+0.1*(rxBuffer[3]-'0')+0.01*(rxBuffer[4]-'0')+0.001*(rxBuffer[5]-'0');
			else
				right_motor_pid.Ki=rxBuffer[2]-'0'+0.1*(rxBuffer[3]-'0')+0.01*(rxBuffer[4]-'0')+0.001*(rxBuffer[5]-'0');
		}
		else if(rxBuffer[0]=='d'){
			if(rxBuffer[1]=='l')
			  left_motor_pid.Kd=rxBuffer[2]-'0'+0.1*(rxBuffer[3]-'0')+0.01*(rxBuffer[4]-'0')+0.001*(rxBuffer[5]-'0');
			else
				left_motor_pid.Kd=rxBuffer[2]-'0'+0.1*(rxBuffer[3]-'0')+0.01*(rxBuffer[4]-'0')+0.001*(rxBuffer[5]-'0');
		}
	  /*else if(rxBuffer[0]=='t'){
			left_motor_pid.Ts=rxBuffer[1]-'0'+0.1*(rxBuffer[2]-'0')+0.01*(rxBuffer[3]-'0')+0.001*(rxBuffer[4]-'0');
		}*/
    for(uint16_t i=0; i<RX_CMD_LEN; i++) 
      proBuffer[i]=rxBuffer[i];   //复制接收到的指令字符串 
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);  //开启IDLE中断 
  } 
}
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) 
{ 
RTC_TimeTypeDef sTime; 
RTC_DateTypeDef sDate; 
if (HAL_RTC_GetTime(hrtc, &sTime,  RTC_FORMAT_BIN) == HAL_OK) 
{ 
HAL_RTC_GetDate(hrtc, &sDate,  RTC_FORMAT_BIN); //必须读取日期 
uint8_t  timeStr[10]; //时间字符串 
timeStr[0] = (sTime.Hours / 10) + '0';
timeStr[1] = (sTime.Hours % 10) + '0';
timeStr[2] = ':';
timeStr[3] = (sTime.Minutes / 10) + '0';
timeStr[4] = (sTime.Minutes % 10) + '0';
timeStr[5] = ':';
timeStr[6] = (sTime.Seconds / 10) + '0';
timeStr[7] = (sTime.Seconds % 10) + '0';
timeStr[8] = '\0'; //计算时、分钟、秒的代码
//LCD_ShowString( 30, 50, timeStr); 
if (isUploadTime)   //变量isUploadTime在文件usart.c中定义 
HAL_UART_Transmit_IT(&huart1,timeStr,sizeof(timeStr)); 
} 
}
/* USER CODE END 1 */
