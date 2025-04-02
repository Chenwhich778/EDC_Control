#include "motor.h"
#include "encoder.h"
// 定义编码器实例
MotorEncoder leftMotorEncoder;
MotorEncoder rightMotorEncoder;

/**
 * @brief 编码器初始化函数
 */
void Encoder_Init(void)
{
    // 初始化左右电机编码器
    Motor_Init(&leftMotorEncoder);
    Motor_Init(&rightMotorEncoder);
    
    // 此处应添加霍尔传感 器相关的GPIO初始化
    // 和中断配置代码，根据实际硬件连接情况
}

/**
 * @brief 霍尔传感器中断回调函数示例
 *        需要在对应的GPIO中断处理函数中调用
 * @param motor 指定是哪个电机的编码器
 */
void Encoder_PulseCallback(MotorEncoder* encoder)
{
    // 脉冲计数增加
    encoder->pulseCount++;
    
    // 计算圈数和距离
    Motor_CalculateRevolutions(encoder);
    Motor_CalculateDistance(encoder);
}

/**
 * @brief 获取左电机行驶距离
 * @return 行驶距离，单位mm
 */
float Encoder_GetLeftDistance(void)
{
    return leftMotorEncoder.distance;
}

/**
 * @brief 获取右电机行驶距离
 * @return 行驶距离，单位mm
 */
float Encoder_GetRightDistance(void)
{
    return rightMotorEncoder.distance;
}

/**
 * @brief 重置电机编码器计数
 */
void Encoder_Reset(void)
{
    Motor_Reset(&leftMotorEncoder);
    Motor_Reset(&rightMotorEncoder);
}
