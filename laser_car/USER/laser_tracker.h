// laser_tracker.h（简化版）
#ifndef LASER_TRACKER_H
#define LASER_TRACKER_H

#include "servo_protocol.h"

extern int servo_x ;  // 水平舵机初始值 (1900-2100)
extern int servo_y;  // 垂直舵机初始值 (2000-2100)
extern int last_obj_x;  // 上一次物体X坐标
extern int last_obj_y;  // 上一次物体Y坐标
extern int lock_count;    // 锁定计数器
extern int move_counter;  // 移动间隔计数器
extern int consecutive_steps; // 连续移动计数

	
#endif

