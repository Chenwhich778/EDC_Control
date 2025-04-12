/*includes----------------------------*/
#include "mpu6050.h"
/*includes----------------------------*/
/*---------------------------------------*/
/*struct-----------------------------*/

/*struct-----------------------------*/
/*-------------------------------------------*/
/*变量------------------------------------*/
//angle
extern float origine_angle;
extern float target_angle;
extern float angle;
extern float pre_angle;
extern float angle_correct;
extern MPU6050_t MPU6050;         // MPU6050数据结构
extern uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
extern void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *Data);
/*变量------------------------------------*/
/*----------------------------------------------*/
/*函数---------------------------------------*/
//angle_pid计算
void CalculateYaw_Filtered(float gyro_z, float dt);
/*函数--------------------------------------------*/