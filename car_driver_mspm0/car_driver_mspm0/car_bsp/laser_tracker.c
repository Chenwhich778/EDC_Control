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
#define KP_X      0.1f  // 比例系数（较大值）
#define KI_X      0.004f  // 积分系数
#define KD_X      0.08f // 微分系

// 垂直方向PID参数（较小系数）
#define KP_Y      0.009f  // 比例系数（较小值）
#define KI_Y      0.0005f
#define KD_Y      0.000f

// 系统全局变量
SystemMode system_mode = MODE_RADAR_SCANNING;   // 当前系统模式
int radar_servo_position = 3800;               // 雷达舵机初始位置（中位）

int buzzer_mark = 0;       // 蜂鸣器触发标记
int R_C = 0;               // 保留变量
int target_position = 0;    // 目标位置

int servo_x = 3800;        // 水平舵机初始值
int servo_y = 2250;        // 垂直舵机初始值
int last_obj_x = CAMERA_CENTER_X;  // 最后物体X坐标（初始为图像中心）
int last_obj_y = CAMERA_CENTER_Y;  // 最后物体Y坐标
int lock_count = 0;        // 锁定计数器
int Lock_count = 0;        // 高级锁定计数器
int temp = 0;              // 临时计数器
int move_counter = 0;      // 移动计数器
int consecutive_steps = 0; // 连续移动步数
bool allow_move = 1;       // 允许移动标志

// 雷达预瞄准控制变量
int radar_aiming = 0;          // 是否正在预瞄准
int radar_aiming_counter = 0;  // 预瞄准倒计时
float last_radar_angle = 0.0f; // 上一次雷达瞄准角度（弧度）
bool lock_flag=false;
extern float adjust;

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

FineServo fine_servo_x = {3800, 0};  // 水平精细控制
FineServo fine_servo_y = {2250, 0};  // 垂直精细控制

// 数值限幅函数
static int constrain(int value, int min, int max) {
    return (value < min) ? min : ((value > max) ? max : value);
}

static float constrain_float(float value, float min, float max) {
    return (value < min) ? min : ((value > max) ? max : value);
}

// // 检查角度偏差是否超出范围（π/6）
// static int is_angle_out_of_range(float current_angle) {
//     float angle_diff = fabs(current_angle - last_radar_angle);
    
//     // 处理角度环绕
//     if (angle_diff > M_PI) {
//         angle_diff = 2 * M_PI - angle_diff;
//     }
    
//     // 判断是否超出π/6范围（30度）
//     return (angle_diff > M_PI/18);
// }

// // 雷达预瞄准函数
// void radar_pre_aim(float enemy_x, float enemy_y) {
//     // 计算目标角度（相对于正前方）
//     float angle_rad = atan2(-enemy_y, -enemy_x);
    
//     // 判断是否需要启动预瞄准
//     if (!radar_aiming && (system_mode == MODE_RADAR_SCANNING || is_angle_out_of_range(angle_rad))) {
//         radar_aiming = 1;
//         radar_aiming_counter = 200;  // 设置预瞄准时长
//         last_radar_angle = angle_rad;  // 记录当前角度
//         system_mode = MODE_RADAR_SCANNING;
//     }
    
//     // 处理预瞄准过程
//     if (radar_aiming) {
//         // 倒计时递减
//         radar_aiming_counter--;
        
//         // 角度转换为舵机位置（0-4095）
//         float angle_deg = angle_rad * 180.0f / M_PI;
        
//         if (angle_deg > 0) {
//             target_position = (int)((270.0f - angle_deg) * 4096.0f / 360.0f) % 4096 - 148;
//         } else {
//             target_position = (int)((-90.0f - angle_deg) * 4096.0f / 360.0f) % 4096 - 148;
//         }
        
//         // 位置限幅
//         if (target_position < 1000) target_position = 1000;
//         if (target_position > 3200) target_position = 3200;
        
//         // 计算目标距离
//         float distance = sqrt(enemy_x * enemy_x + enemy_y * enemy_y);
        
//         // 根据距离设置移动速度
//         int move_speed = 1000;  // 默认速度
//         if (distance < RADAR_MIN_RANGE) {
//             move_speed = 1500;  // 目标过近，快速响应
//         } else if (distance > RADAR_MAX_RANGE * 0.7) {
//             move_speed = 1000;  // 目标较远，慢速移动
//         }
        
//         // 更新雷达舵机位置
//         radar_servo_position = target_position;
        
//         // 控制水平舵机
//         Servo_SetPosition(1, target_position, move_speed);
        
//         // 控制垂直舵机（保持中位）
//         Servo_SetPosition(2, servo_y, move_speed);
        
//         // 检查预瞄准是否完成
//         if (radar_aiming_counter <= 0) {
//             radar_aiming = 0;
//             system_mode = MODE_CAMERA_TRACKING;  // 切换到摄像头追踪模式
            
//             // 初始化追踪状态
//             velocity_x = 0.0f;
//             velocity_y = 0.0f;
//             fine_servo_x.base = target_position;
//             fine_servo_x.fine = 0;
//             servo_x = target_position;
//         }
        
//         // 重置锁定计数器
//         lock_count = 0;
//         return;
//     }
    
//     // 非预瞄准状态处理（雷达扫描模式）
//     if (system_mode == MODE_RADAR_SCANNING) {
//         // 此处可添加扫描模式的具体行为
//     }
// }

// 摄像头追踪控制函数
void control_camera(int obj_x, int obj_y) {
    // // 非追踪模式直接返回
    // if (system_mode != MODE_CAMERA_TRACKING) {
    //      allow_move = 0;
    //     return;
    // }
    // else allow_move = 1;

    
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
    int error_y = CAMERA_CENTER_Y - predicted_y;
    
    // 微调步长设置
    const float FINE_STEP_SIZE = 0.2f;
    float fine_step_x = 0;
    float fine_step_y = 0;
    
    // 锁定阈值
    int LOCK_THRESHOLD_X = 5;
    int LOCK_THRESHOLD_Y = 5;

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

    // 锁定状态判断
    if (abs(error_x) <= LOCK_THRESHOLD_X && abs(error_y) <= LOCK_THRESHOLD_Y) {
        lock_count++;
        if (lock_count >= 2) {
            // 锁定状态保持
            consecutive_steps = 0;
            velocity_x = 0.0f;
            velocity_y = 0.0f;
            lock_flag=true;
            return;
        }
    } else {
        lock_count = 0;
    }
    
    // 水平方向PID控制
    if (allow_move && abs(error_x) > LOCK_THRESHOLD_X) {
        float P = KP_X * error_x;
        integral_x += error_x;
        float I = KI_X * integral_x;
        float D = KD_X * (error_x - prev_error_x);
        constrain(integral_x,-10,10);

        float pid_output = P + I + D;
        prev_error_x = error_x;
        fine_step_x = -pid_output;
        constrain_float(fine_step_x, 0, 15*FINE_STEP_SIZE);
    } else {
        integral_x = 0;
    }

    // 垂直方向PID控制
    if (allow_move && abs(error_y) > LOCK_THRESHOLD_Y) {
        float P = KP_Y * error_y;
        integral_y += error_y;
        float I = KI_Y * integral_y;
        float D = KD_Y * (error_y - prev_error_y);
        
        float pid_output = P + I + D;
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

            fine_servo_x.base = constrain(fine_servo_x.base + actual_step, 100,4000);
            fine_servo_x.fine -= actual_step;
            consecutive_steps++;
            servo_x = fine_servo_x.base;
        }
    }
    
    if (fine_step_y != 0) {
        fine_servo_y.fine += fine_step_y;
        if (fabs(fine_servo_y.fine) >= 1.0f) {
            int actual_step = (int)truncf(fine_servo_y.fine);
            fine_servo_y.base = constrain(fine_servo_y.base + actual_step, 2100, 2350);
            fine_servo_y.fine -= actual_step;
            consecutive_steps++;
            servo_y = fine_servo_y.base;
        }
    }
    
    // 执行舵机控制
    static int last_servo_x = 3800;
    static int last_servo_y = 2250;
    
    if (servo_x != last_servo_x || servo_y != last_servo_y) {
        if (servo_x != last_servo_x) {           
            last_servo_x = servo_x;
        }
        if (servo_y != last_servo_y) {
            Servo_SetPosition(2, servo_y, 200);
            last_servo_y = servo_y;
        }
    } else {
        consecutive_steps = 0;
    }
}
