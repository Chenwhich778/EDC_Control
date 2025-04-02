#ifndef __ENCODER_H
#define __ENCODER_H

#include "motor.h"

// 函数声明
void Encoder_Init(void);
void Encoder_PulseCallback(MotorEncoder* encoder);
float Encoder_GetLeftDistance(void);
float Encoder_GetRightDistance(void);
void Encoder_Reset(void);

#endif /* __ENCODER_H */
