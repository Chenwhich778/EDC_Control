#include "motor.h"

/**
 * @brief  初始化电机编码器
 * @param  encoder: 电机编码器结构体指针
 * @retval None
 */
void Motor_Init(MotorEncoder* encoder)
{
    if (encoder != NULL)
    {
        encoder->pulseCount = 0;
        encoder->revolutions = 0.0f;
        encoder->distance = 0.0f;
    }
}

/**
 * @brief  更新编码器脉冲计数
 * @param  encoder: 电机编码器结构体指针
 * @param  newCount: 新的脉冲计数值
 * @retval None
 */
void Motor_UpdatePulseCount(MotorEncoder* encoder, int32_t newCount)
{
    if (encoder != NULL)
    {
        encoder->pulseCount = newCount;
        encoder->revolutions = Motor_CalculateRevolutions(encoder);
        encoder->distance = Motor_CalculateDistance(encoder);
    }
}

/**
 * @brief  计算电机转动的圈数
 * @param  encoder: 电机编码器结构体指针
 * @retval 转动的圈数
 */
float Motor_CalculateRevolutions(MotorEncoder* encoder)
{
    if (encoder != NULL)
    {
        return (float)(encoder->pulseCount) / ENCODER_PULSES_PER_REVOLUTION;
    }
    return 0.0f;
}

/**
 * @brief  计算轮子行驶的距离
 * @param  encoder: 电机编码器结构体指针
 * @retval 行驶距离(mm)
 */
float Motor_CalculateDistance(MotorEncoder* encoder)
{
    if (encoder != NULL)
    {
        // 距离 = 圈数 * 周长
        float circumference = PI * WHEEL_DIAMETER;
        return encoder->revolutions * circumference;
    }
    return 0.0f;
}

/**
 * @brief  重置电机编码器数据
 * @param  encoder: 电机编码器结构体指针
 * @retval None
 */
void Motor_Reset(MotorEncoder* encoder)
{
    if (encoder != NULL)
    {
        encoder->pulseCount = 0;
        encoder->revolutions = 0.0f;
        encoder->distance = 0.0f;
    }
}
