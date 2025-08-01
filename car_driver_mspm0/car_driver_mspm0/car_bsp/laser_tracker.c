// laser_tracker.c - 雷达预瞄准与摄像头追踪系统
#include "laser_tracker.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 全局变量用于存储PID状态（实际使用时应放入系统初始化）
float integral_x = 0, integral_y = 0;          // 积分项
float prev_error_x = 0, prev_error_y = 0;      // 上一次误差

// 水平方向PID参数（较大系数）
#define KP_X      0.3f  // 比例系数（较大值）
#define KI_X      0.02f  // 积分系数
#define KD_X      0.006f // 微分系

// 垂直方向PID参数（较小系数）
#define KP_Y      0.09f  // 比例系数（较小值）
#define KI_Y      0.001f
#define KD_Y      0.001f

// 系统全局变量
SystemMode system_mode = MODE_RADAR_SCANNING;   // 当前系统模式
int radar_servo_position = 3800;               // 雷达舵机初始位置（中位）

int buzzer_mark = 0;       // 蜂鸣器触发标记
int R_C = 0;               // 保留变量
int target_position = 0;    // 目标位置

int servo_x = 700;        // 水平舵机初始值
int servo_y = 2250;        // 垂直舵机初始值
int last_obj_x = CAMERA_CENTER_X;  // 最后物体X坐标（初始为图像中心）
int last_obj_y = CAMERA_CENTER_Y;  // 最后物体Y坐标
int lock_count = 0;        // 锁定计数器
int Lock_count = 0;        // 高级锁定计数器
int temp = 0;              // 临时计数器
int move_counter = 0;      // 移动计数器
int consecutive_steps = 0; // 连续移动步数
bool allow_move = 1;       // 允许移动标志


bool lock_flag=false;
extern float adjust;
extern int distance;
extern uint8_t laser_flag;

// 运动预测参数
const float PREDICTION_FACTOR = 0.3f;          // 预测因子
const int MIN_MOVE_FOR_PREDICTION = 10;        // 触发预测的最小移动量
float velocity_x = 0.0f;                       // X轴速度
float velocity_y = 0.0f;                       // Y轴速度

// 精细舵机控制结构体
typedef struct {
    int base;    // 基础位置
    float fine;  // 微调量
} FineServo;

FineServo fine_servo_x = {700, 0};  // 水平精细控制
FineServo fine_servo_y = {2250, 0};  // 垂直精细控制

// 数值限幅函数
static int constrain(int value, int min, int max) {
    return (value < min) ? min : ((value > max) ? max : value);
}

static float constrain_float(float value, float min, float max) {
    return (value < min) ? min : ((value > max) ? max : value);
}




int laser_y = CAMERA_CENTER_Y;
float distance_ratio = 0;
float max_adjustment_range = 0;
float y_offset = 0;
int adjusted_y;

// 摄像头追踪控制函数
void control_camera(int obj_x, int obj_y) {
    
    // distance_ratio = (float)distance / 500;
    // max_adjustment_range = 480 * 0.2;
    // y_offset = (distance_ratio - 1) * max_adjustment_range * 0.25;
    // adjusted_y = laser_y - (int)y_offset;
    if(distance >= 600){
    int interval = (distance - 600) / 100;  // 整数除法
    laser_y = 266 - 3 * (interval + 1);
} else {
    laser_y = 266;
}
    constrain(laser_y, 250, 266);

    
    // 计算目标移动量
    int delta_x = obj_x - last_obj_x;
    int delta_y = obj_y - last_obj_y;

    // 更新历史位置
    last_obj_x = obj_x;
    last_obj_y = obj_y;
    
    // 计算目标速度（低通滤波）
    const float ALPHA = 0.15f;
    velocity_x = ALPHA * delta_x + (1 - ALPHA) * velocity_x;
    velocity_y = ALPHA * delta_y + (1 - ALPHA) * velocity_y;
    
    // 运动预测
    int predicted_x = obj_x;
    int predicted_y = obj_y;
    
    if (abs(delta_x) > MIN_MOVE_FOR_PREDICTION || abs(delta_y) > MIN_MOVE_FOR_PREDICTION) {
        predicted_x = obj_x + (int)(PREDICTION_FACTOR * velocity_x);
        predicted_y = obj_y + (int)(PREDICTION_FACTOR * velocity_y);
    }
    
    // 计算与图像中心的误差
    int error_x = CAMERA_CENTER_X - predicted_x;
    int error_y = laser_y - predicted_y;
    
    // 微调步长设置
    const float FINE_STEP_SIZE = 0.2f;
    float fine_step_x = 0;
    float fine_step_y = 0;
    
    // 锁定阈值
    int LOCK_THRESHOLD_X = 3;
    int LOCK_THRESHOLD_Y = 3;

    float pre_out_x = 0;
    float pre_out_y = 0;
    // 高级锁定判断（基于目标区域）
    // if (y1<(257-(y3-y1)/50-2)&&y3>(257+(y3-y1)/50+2)&&x1<(371-(x3-x1)/50-2)&&x3>(371+(x3-x1)/50+2)) {
    //     Lock_count++;
    //     temp = 0;
    // } else {
    //     temp++;
    //     if (temp >= 80) {
    //         if(Lock_count<205) Lock_count = 0;
    //     }
    // }

    // // 蜂鸣器触发逻辑
    // if (Lock_count >= 205 && Lock_count < 300) {
    //     buzzer_mark = 1;
    //     Lock_count++;
    // }
    // if(Lock_count>=300){
    //     Lock_count=0;
    //     buzzer_mark=0;
    // }

if (abs(error_x) <= 5 && abs(error_y) <= 5)
       { 
        Lock_count++;
        if (Lock_count >= 2){
         if(laser_flag  ==0)
            laser_flag=1;
       }
       }
else {
        Lock_count = 0;
    }

    // // 锁定状态判断
    // if (abs(error_x) <= LOCK_THRESHOLD_X && abs(error_y) <= LOCK_THRESHOLD_Y) {
    //     lock_count++;
    //     if (lock_count >= 2) {
    //         // 锁定状态保持
    //         consecutive_steps = 0;
    //         velocity_x = 0.0f;
    //         velocity_y = 0.0f;
    //         lock_flag=true;
    //     }
    // } else {
    //     lock_count = 0;
    // }
    
    // 水平方向PID控制
    if (allow_move && abs(error_x) > LOCK_THRESHOLD_X) {
        float P = KP_X * error_x;
        integral_x += error_x;
        float I = KI_X * integral_x;
        float D = KD_X * (error_x - prev_error_x);
        constrain(integral_x,-100,100);

        float pid_output = (P + I + D)*0.8+0.2*pre_out_x;
        pre_out_x = pid_output;
        prev_error_x = error_x;
        fine_step_x = -pid_output;
        constrain_float(fine_step_x, 0, 10*FINE_STEP_SIZE);
    } else {
        integral_x = 0;
    }

    // 垂直方向PID控制
    if (allow_move && abs(error_y) > LOCK_THRESHOLD_Y) {
        float P = KP_Y * error_y;
        integral_y += error_y;
        float I = KI_Y * integral_y;
        float D = KD_Y * (error_y - prev_error_y);
        
        float pid_output =( P + I + D)*0.8+0.2*pre_out_y;
        pre_out_y = pid_output;
        prev_error_y = error_y;
        fine_step_y = pid_output; 
        constrain_float(fine_step_x, 0, 2*FINE_STEP_SIZE);
    } else {
        integral_y = 0;
    }
    
    // 应用精细控制
    if (fine_step_x != 0) {
        fine_servo_x.fine += fine_step_x;
        if (fabs(fine_servo_x.fine) >= 1.0f) {
            int actual_step = (int)truncf(fine_servo_x.fine);

            fine_servo_x.base = constrain(fine_servo_x.base + actual_step, 100,10000);
            fine_servo_x.fine -= actual_step;
            consecutive_steps++;
            servo_x = fine_servo_x.base;
        }
    }
    
    if (fine_step_y != 0) {
        fine_servo_y.fine += fine_step_y;
        if (fabs(fine_servo_y.fine) >= 1.0f) {
            int actual_step = (int)truncf(fine_servo_y.fine);
            fine_servo_y.base = constrain(fine_servo_y.base + actual_step, 2000, 2450);
            fine_servo_y.fine -= actual_step;
            consecutive_steps++;
            servo_y = fine_servo_y.base;

        }
    }
    
    // 执行舵机控制
    static int last_servo_x = 700;
    static int last_servo_y = 2250;
    
    if (servo_x != last_servo_x || servo_y != last_servo_y) {
        if (servo_x != last_servo_x) {           
            last_servo_x = servo_x;
        }
        if (servo_y != last_servo_y) {
            last_servo_y = servo_y;
        }
    } else {
        consecutive_steps = 0;
    }
}
