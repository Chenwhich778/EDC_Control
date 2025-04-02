#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include <stdint.h>

// 电机及编码器参数定义
#define ENCODER_PULSES_PER_REVOLUTION 11   // 霍尔传感器每圈脉冲数（根据实际情况调整）
#define WHEEL_DIAMETER               65.0f  // 轮子直径，单位mm
#define PI                           3.14159f

// 电机控制结构体
typedef struct {
    int32_t pulseCount;       // 脉冲计数
    float revolutions;        // 转动圈数
    float distance;           // 行驶距离，单位mm
} MotorEncoder;

// 函数声明
void Motor_Init(MotorEncoder* encoder);
void Motor_UpdatePulseCount(MotorEncoder* encoder, int32_t newCount);
float Motor_CalculateRevolutions(MotorEncoder* encoder);
float Motor_CalculateDistance(MotorEncoder* encoder);
void Motor_Reset(MotorEncoder* encoder);

#endif /* __MOTOR_H */
