// laser_tracker.h
#ifndef LASER_TRACKER_H
#define LASER_TRACKER_H

#include "servo_protocol.h"

// 系统工作模式
typedef enum {
    MODE_RADAR_SCANNING,   // 雷达扫描模式
    MODE_CAMERA_TRACKING   // 摄像头追踪模式
} SystemMode;

// 雷达参数
#define RADAR_MAX_RANGE 100.0f  // 雷达最大探测距离(m)
#define RADAR_MIN_RANGE 1.0f    // 雷达最小探测距离(m)

// 摄像头中心位置
#define CAMERA_CENTER_X 359
#define CAMERA_CENTER_Y 221

// 全局状态
extern int servo_x;        // 水平舵机初始值
extern int servo_y;
extern bool lock_flag;
extern SystemMode system_mode;
extern int radar_servo_position;  // 雷达预锁定舵机位置
        
extern int radar_aiming_counter;  // 设置预瞄准计时器

extern int buzzer_mark;  // 蜂鸣器标志
extern int Lock_count;

// 函数声明
void radar_pre_aim(float enemy_x, float enemy_y);
void control_camera(int obj_x, int obj_y);

#endif