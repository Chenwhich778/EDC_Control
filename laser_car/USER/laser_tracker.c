// laser_tracker.c（修改同步控制为顺序控制）
#include "laser_tracker.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

// 辅助函数：数值限幅
static int constrain(int value, int min, int max) {
    return (value < min) ? min : ((value > max) ? max : value);
}

static float Constrain(float value, float min, float max) {
    return (value < min) ? min : ((value > max) ? max : value);
}

// 全局变量（需在函数外部声明）
int servo_x = 2000;  // 水平舵机初始值 (1900-2100)
int servo_y = 2050;  // 垂直舵机初始值 (2000-2100)
int last_obj_x = 320;  // 上一帧物体x坐标
int last_obj_y = 290;  // 上一帧物体y坐标
int lock_count = 0;    // 锁定计数器
int move_counter = 0;  // 移动间隔计数器
int consecutive_steps = 0; // 连续移动计数

// 精细控制结构（用于亚步长移动）
typedef struct {
    int base;      // 基础舵机位置
    float fine;    // 亚步长累积值
} FineServo;

FineServo fine_servo_x = {2000, 0};
FineServo fine_servo_y = {2050, 0};

void control_camera(int obj_x, int obj_y) {
    // 目标固定点坐标（相机中心）
    const int TARGET_X = 325;
    const int TARGET_Y = 268;
    
    // 计算图像坐标误差
    int error_x = TARGET_X - obj_x;
    int error_y = obj_y - TARGET_Y;
    
    // 计算物体移动方向（用于预测）
    int delta_x = obj_x - last_obj_x;
    int delta_y = obj_y - last_obj_y;
    last_obj_x = obj_x;
    last_obj_y = obj_y;
    
    // 超小步进控制参数 - 减小步幅到0.2舵机单位
    const float FINE_STEP_SIZE = 0.2f;  // 精细步长（从0.25减小到0.2）
    float fine_step_x = 0;
    float fine_step_y = 0;
    
    // 移动间隔控制（每3帧移动一次）
    move_counter = (move_counter + 1) % 4;
    bool allow_move = (move_counter == 0);
    
    // 默认锁定阈值
    int LOCK_THRESHOLD_X = 5;
    int LOCK_THRESHOLD_Y = 5;
    // 当识别到的尺寸小于28时，阈值放宽到6
    if (info.size < 28.0f) {
        LOCK_THRESHOLD_X = 6;
        LOCK_THRESHOLD_Y = 6;
    }

    // 检查是否达到锁定条件
    if (abs(error_x) <= LOCK_THRESHOLD_X && abs(error_y) <= LOCK_THRESHOLD_Y) {
        lock_count++;
        // 只需连续2帧在锁定区域内就认为锁定
        if (lock_count >= 2) {
            // 锁定状态不进行任何移动
            consecutive_steps = 0;
            return;
        }
    } else {
        lock_count = 0;  // 误差超出阈值，重置锁定计数
    }
    
    // 水平方向微步控制（仅在允许移动时）
    if (allow_move && abs(error_x) > LOCK_THRESHOLD_X) {
        fine_step_x = (error_x > 0) ? -FINE_STEP_SIZE : FINE_STEP_SIZE;
    }
    
    // 垂直方向微步控制（仅在允许移动时）
    if (allow_move && abs(error_y) > LOCK_THRESHOLD_Y) {
        fine_step_y = (error_y > 0) ? FINE_STEP_SIZE : -FINE_STEP_SIZE;
    }
    
    // 应用精细控制
    if (fine_step_x != 0) {
        fine_servo_x.fine += fine_step_x;
        
        // 累积到整数步时更新实际舵机位置
        if (fabs(fine_servo_x.fine) >= 1.0f) {
            int actual_step = (int)truncf(fine_servo_x.fine);
            fine_servo_x.base = constrain(fine_servo_x.base + actual_step, 1800, 2200);
            fine_servo_x.fine -= actual_step;
            consecutive_steps++;
            
            // 更新全局舵机位置
            servo_x = fine_servo_x.base;
        }
    }
    
    if (fine_step_y != 0) {
        fine_servo_y.fine += fine_step_y;
        
        // 累积到整数步时更新实际舵机位置
        if (fabs(fine_servo_y.fine) >= 1.0f) {
            int actual_step = (int)truncf(fine_servo_y.fine);
            fine_servo_y.base = constrain(fine_servo_y.base + actual_step, 1950, 2200);
            fine_servo_y.fine -= actual_step;
            consecutive_steps++;
            
            // 更新全局舵机位置
            servo_y = fine_servo_y.base;
        }
    }
    
    // 长时间移动保护（防止无限调整）
    if (consecutive_steps > 30) {
        // 连续移动30次后强制暂停
        consecutive_steps = 0;
        lock_count = 2;  // 强制进入锁定状态
        return;
    }
    
    // 发送指令（仅当有实际舵机位置变化时）
    static int last_servo_x = 2000;
    static int last_servo_y = 2050;
    
    if (servo_x != last_servo_x || servo_y != last_servo_y) {
        if (servo_x != last_servo_x) {
            Servo_SetPosition(1, servo_x, 2);  // 水平舵机（增加时间减少震荡）
            last_servo_x = servo_x;
        }
        if (servo_y != last_servo_y) {
            Servo_SetPosition(2, servo_y, 2);  // 垂直舵机
            last_servo_y = servo_y;
        }
        
    } else {
        consecutive_steps = 0;  // 无移动时重置计数
    }
}