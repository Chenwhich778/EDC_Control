/*includes----------------------------*/
#include "angle.h"
#include "math.h"
/*includes----------------------------*/
/*---------------------------------------*/
/*struct-----------------------------*/

/*struct-----------------------------*/
/*-------------------------------------------*/
/*变量------------------------------------*/
//angle
float origine_angle=0.0f;
float target_angle=0.0f;
float angle=0.0f;
float pre_angle=0.0f;
float angle_correct=4.8;
MPU6050_t MPU6050;         // MPU6050数据结构
extern uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
extern void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *Data);
/*变量------------------------------------*/
/*----------------------------------------------*/
/*函数---------------------------------------*/

void CalculateYaw_Filtered(float gyro_z, float dt)
{
    // 静态变量保存yaw和bias
    static float yaw = 0.0;
    static float bias = 0.0;
    
    // 如果测量值很小，认为设备静止，可以更新零偏
    const float threshold = 2.5; // 阈值，根据实际情况调整
    if(fabs(gyro_z) < threshold)
    {
        bias = gyro_z;
    }
    
    // 用减去偏置后的角速度积分计算yaw
    float corrected_rate = gyro_z - bias;
    yaw += corrected_rate * dt;
    angle=yaw;
    return;
}
/*函数--------------------------------------------*/