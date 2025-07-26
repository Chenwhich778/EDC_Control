#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <stdint.h>

// 读取7路数字灰度传感器（0=白线，1=黑线）
void Read_Grayscale(uint8_t values[7]);

#endif