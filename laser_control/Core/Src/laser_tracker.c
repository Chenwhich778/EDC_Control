// laser_tracker.c（修改同步控制为顺序控制）
#include "laser_tracker.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>


extern int circle3;
 float test = 3;
 int servo3 = 3700 ;
 int servo4 = 805 ;

// 初始化跟踪器
void init_tracker(LaserTracker *tracker, Point vertices[4]) {
    for (int i = 0; i < 4; i++) {
        tracker->vertices[i] = vertices[i];
    }
    memset(tracker->path_points, 0, sizeof(tracker->path_points));
    tracker->path_size = 0;
    tracker->current_target = 0;
    tracker->laser_pos = tracker->vertices[0]; // 初始位置设为左上角顶点
    tracker->servo1_pos = 2970; // 舵机1初始位置
    tracker->servo2_pos = 3290; // 舵机2初始位置
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

static float Constrain(float value, float min, float max) {
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

		//  检查是否到达目标点
    float distance2 = error_x * error_x + error_y * error_y;
		float threshold = 500.0f ; // 速度相关阈值
		
    // 比例控制参数 (需要根据实际硬件调整)
    const float KP = 0.069f;
    
    // 计算比例控制输出
    float move_x = KP * error_x;
    float move_y = KP * error_y;
		
		int rate1 = 50;
		int rate2 = 50;
		
		if((tracker->current_target >0 && tracker->current_target <=25)||(tracker->current_target >50 && tracker->current_target <=75))
		{
			rate1 = 1000;
			rate2 = 350;
		}
    else    {
			rate1 = 350;
			rate2 = 1000;
		}
		
    move_x = Constrain( move_x, -1, 1);
		move_y = Constrain( move_y, -1, 1);
		
		
		
    //  更新舵机位置（使用实际范围）
    tracker->servo1_pos = constrain(tracker->servo1_pos + (int)move_x, 2700, 3400);
		
    tracker->servo2_pos = constrain(tracker->servo2_pos + (int)move_y, 3100, 3600);
    
    //  发送控制指令
    Servo_SetPosition(1, tracker->servo1_pos, rate1);
		HAL_Delay(10);
    Servo_SetPosition(2, tracker->servo2_pos, rate2);
    
    
		
		test = tracker->servo1_pos ;//测试
		
    if (distance2 < threshold) { // 稍微增大阈值
        tracker->current_target = (tracker->current_target + 1) % tracker->path_size;
    }
    
		 
		if   (tracker->current_target == 0 ) {
        return;}
}

void control_green_light(int a, int b, int c, int d) {
    // 计算当前绿点与红点的误差
    float error_x = a - c;
    float error_y = b - d;

     // 比例控制参数 (需要根据实际硬件调整)
    const float KP = 0.07f;
    
    // 计算比例控制输出
    float move_x = KP * error_x;
    float move_y = KP * error_y;
       
    move_x = Constrain( move_x, -1, 1);
		move_y = Constrain( move_y, -1, 1);
    // 更新舵机位置（确保在安全范围内）
	
	//  检查是否到达目标点
    float distance2 = error_x * error_x + error_y * error_y;
    if (distance2 < 60.0f||circle3 == 0||a == 0 || b == 0) { 
			move_x = 0;
			move_y = 0;
    }
        servo3 = constrain(servo3 + (int)move_x, 3400, 4050); // 舵机1控制绿点x方向
        servo4 = constrain(servo4 + (int)move_y, 650, 1150); // 舵机2控制绿点y方向

    // 发送舵机控制指令
    Servo_SetPosition(3, servo3, 1000); // 绿光舵机通道为3
    Servo_SetPosition(4, servo4, 1000); // 绿光舵机通道为4

	  
		 
}
