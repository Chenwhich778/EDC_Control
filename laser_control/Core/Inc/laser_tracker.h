// laser_tracker.h（简化版）
#ifndef LASER_TRACKER_H
#define LASER_TRACKER_H

#include "servo_protocol.h"
#include <stdint.h>
#include "tim.h"

extern  float test;
#define MAX_PATH_POINTS 100  // 预定义最大路径点数
// 坐标点结构体
typedef struct {
    float x;
    float y;
} Point;

// 系统参数
typedef struct {
    Point vertices[4];   // 方框四个顶点坐标
    Point laser_pos;     // 激光点当前位置（摄像头坐标）
    Point target_pos;    // 当前目标位置（摄像头坐标）
    int current_target;  // 当前目标点索引
    Point path_points[MAX_PATH_POINTS];  // 静态路径点数组
    int path_size;       // 路径点数量
    int servo1_pos;      // 舵机1目标位置（直接控制值）
    int servo2_pos;      // 舵机2目标位置（直接控制值）
} LaserTracker;

// 函数声明
void init_tracker(LaserTracker *tracker, Point vertices[4]);
void generate_path(LaserTracker *tracker, int segments_per_side);
void update_laser_position(LaserTracker *tracker, Point new_pos);
void control_laser(LaserTracker *tracker);
void control_green_light(int a, int b, int c, int d);
#endif

