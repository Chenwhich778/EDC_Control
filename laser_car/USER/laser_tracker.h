// laser_tracker.h（简化版）
#ifndef LASER_TRACKER_H
#define LASER_TRACKER_H


#include "servo_protocol.h"
#include "stdio.h"
#include "sys.h"
#include "system.h"


// PID控制器参数

#define KP_x 1.0    // 比例系数1.25
#define KI_x 0.08   // 积分系数0.1
#define KD_x 0.14   // 微分系数0.2

#define KP_y 0.5    // 比例系数0.5
#define KI_y 0.04   // 积分系数0.04
#define KD_y 0.08   // 微分系数0.08


// 边界限制
#define MIN_X 0
#define MAX_X 4095
#define MIN_Y (center_y1 - 400)
#define MAX_Y (center_y1 + 400)
#define MAX_INTER_X (MAX_X-MIN_X+1)/KI_x
#define MAX_INTER_Y (MAX_Y-MIN_Y+1)
#define MAX_FINE_ADJUST 2048  // 4096 * 0.5
//变量定义
extern float general_angle;

extern uint16_t servo[4];
extern uint8_t find_target;
extern uint8_t track_flag;
extern float search_delt;
extern uint16_t lose_time;
extern int laser_x;
extern int laser_y;


// 函数声明
void laser_track(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4);
#endif

