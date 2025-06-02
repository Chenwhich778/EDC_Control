// laser_tracker.c（修改同步控制为顺序控制）
#include "laser_tracker.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>


extern int circle2;
 float test = 3;
 int servo3 = 3180 ;
 int servo4 = 840 ;

// 初始化跟踪器
void init_tracker(LaserTracker *tracker, Point vertices[4]) {
    for (int i = 0; i < 4; i++) {
        tracker->vertices[i] = vertices[i];
    }
    memset(tracker->path_points, 0, sizeof(tracker->path_points));
    tracker->path_size = 0;
    tracker->current_target = 0;
    tracker->laser_pos = tracker->vertices[0]; // 初始位置设为左上角顶点
    tracker->servo1_pos = 2800; // 舵机1初始位置
    tracker->servo2_pos = 3458; // 舵机2初始位置
}

void generate_path(LaserTracker *tracker, int segments_per_side) {
    tracker->path_size = 4 * segments_per_side;         // 总点数 = 4边 × 每边分段数
    
    if (tracker->path_size > MAX_PATH_POINTS) {
        tracker->path_size = 0;
        return;
    }
    
    //  生成路径点
    int index = 0;
    for (int side = 0; side < 4; side++) {
        Point start = tracker->vertices[side];
        Point end = tracker->vertices[(side + 1) % 4];
        
        // 生成当前边的点（不包含终点，避免重复）
        for (int i = 0; i < segments_per_side; i++) {
            float t = (float)i / segments_per_side; 
            tracker->path_points[index++] = (Point){
                start.x + t * (end.x - start.x),
                start.y + t * (end.y - start.y)
            };
        }
    }
}

// 更新激光位置
void update_laser_position(LaserTracker *tracker, Point new_pos) {
    tracker->laser_pos = new_pos;
}

// 辅助函数：数值限幅
static int constrain(int value, int min, int max) {
    return (value < min) ? min : ((value > max) ? max : value);
}

void control_laser(LaserTracker *tracker) {
    //  安全检查放在函数开头
    if (tracker->path_size <= 0 || 
        tracker->current_target < 0 || 
        tracker->current_target >= tracker->path_size) {
        return;
    }
    
    // 获取当前目标点
    tracker->target_pos = tracker->path_points[tracker->current_target];
	
    float error_x = tracker->target_pos.x - tracker->laser_pos.x;
    float error_y = tracker->target_pos.y - tracker->laser_pos.y;

    const int STEP = 1;
    int delta1 = (error_x > 0) ? STEP : -STEP;  // x轴误差>0时右移，否则左移
    int delta2 = (error_y > 0) ? STEP : -STEP;  // y轴误差>0时下移（根据坐标系y轴向下）
    
    //  更新舵机位置（使用实际范围）
    tracker->servo1_pos = constrain(tracker->servo1_pos + delta1, 2400, 3100);
    tracker->servo2_pos = constrain(tracker->servo2_pos + delta2, 3300, 3700);
    
    //  发送控制指令
    Servo_SetPosition(1, tracker->servo1_pos, 80);
    Servo_SetPosition(2, tracker->servo2_pos, 75);
    
    //  检查是否到达目标点
    float distance2 = error_x * error_x + error_y * error_y;
		
		test = tracker->servo1_pos ;//测试
		
    if (distance2 < 130.0f) { // 稍微增大阈值
        tracker->current_target = (tracker->current_target + 1) % tracker->path_size;
    }
    
}

void control_green_light(int a, int b, int c, int d) {
    // 计算当前绿点与红点的误差
    float error_x = a - c;
    float error_y = b - d;

    const int STEP = 1; // 跟踪步长，可根据需要调整
    
    // 根据误差方向决定步进方向
    int delta_x = (error_x > 0) ? STEP : -STEP;
    int delta_y = (error_y > 0) ? STEP : -STEP;

    // 更新舵机位置（确保在安全范围内）
	
        servo3 = constrain(servo3 + delta_x, 2800, 3250); // 舵机1控制绿点x方向
        servo4 = constrain(servo4 + delta_y, 800, 950); // 舵机2控制绿点y方向

    // 发送舵机控制指令
    Servo_SetPosition(3, servo3, 60); // 绿光舵机通道为3
    Servo_SetPosition(4, servo4, 60); // 绿光舵机通道为4

	  //  检查是否到达目标点
    float distance2 = error_x * error_x + error_y * error_y;
    if (distance2 < 20.0f) { return;
    }
		 
}
