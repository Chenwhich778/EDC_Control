// laser_tracker.c - 雷达预锁定与摄像头追踪
#include "laser_tracker.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 全局变量
SystemMode system_mode = MODE_RADAR_SCANNING;
int radar_servo_position = 2048;  // 雷达舵机初始位置（正前方）

int buzzer_mark = 0;

int servo_x = 1900;  // 水平舵机初始值
int servo_y = 2200;  // 垂直舵机初始值
int last_obj_x = CAMERA_CENTER_X;  // 必须初始化为图像中心坐标
int last_obj_y = CAMERA_CENTER_Y;  // 必须初始化为图像中心坐标
int lock_count = 0;
int Lock_count = 0;
int temp =0 ;
int move_counter = 0;
int consecutive_steps = 0;

// 新增：雷达预瞄准控制变量
int radar_aiming = 0;          // 是否正在预瞄准
int radar_aiming_counter = 0;  // 预瞄准计时器
float last_radar_angle = 0.0f; // 上一次雷达瞄准的角度（弧度）

// 运动预测参数
const float PREDICTION_FACTOR = 0.3f;
const int MIN_MOVE_FOR_PREDICTION = 10;
float velocity_x = 0.0f;
float velocity_y = 0.0f;

// 精细控制结构
typedef struct {
    int base;
    float fine;
} FineServo;

FineServo fine_servo_x = {1900, 0};
FineServo fine_servo_y = {2200, 0};

// 辅助函数
static int constrain(int value, int min, int max) {
    return (value < min) ? min : ((value > max) ? max : value);
}

static float constrain_float(float value, float min, float max) {
    return (value < min) ? min : ((value > max) ? max : value);
}

// 检查角度偏差是否超出范围（±π/6）
static int is_angle_out_of_range(float current_angle) {
    float angle_diff = fabs(current_angle - last_radar_angle);
    
    // 处理角度环绕问题
    if (angle_diff > M_PI) {
        angle_diff = 2 * M_PI - angle_diff;
    }
    
    // 检查是否超出±π/6范围（30度）
    return (angle_diff > M_PI/8);
}

// 雷达预瞄准函数
void radar_pre_aim(float enemy_x, float enemy_y) {
    // 计算目标角度（相对于正前方）
    float angle_rad = atan2(-enemy_y, -enemy_x);
    int target_position = 0;
    
    // 检查是否需要启动预瞄准
    if (!radar_aiming && (system_mode == MODE_RADAR_SCANNING || is_angle_out_of_range(angle_rad))) {
        radar_aiming = 1;
        radar_aiming_counter = 200;  // 设置预瞄准计时器
        last_radar_angle = angle_rad;  // 保存当前角度
        system_mode = MODE_RADAR_SCANNING;
    }
    
    // 如果正在预瞄准
    if (radar_aiming) {
        // 更新计时器
        radar_aiming_counter--;
        
        // 将角度转换为舵机位置 (0-4095)
        float angle_deg = angle_rad * 180.0f / M_PI;
        
        if (angle_deg > 0) {
            target_position = (int)((270.0f - angle_deg) * 4096.0f / 360.0f) % 4096 - 100;
        } else {
            target_position = (int)((-90.0f - angle_deg) * 4096.0f / 360.0f) % 4096 - 100;
        }
        
        if (target_position < 1000) {
            target_position = 1000;  // 确保舵机位置在0-4095范围内
        }
        if (target_position > 3200) {
            target_position = 3200;  // 确保舵机位置在0-4095范围内
        }
        // 计算到目标的距离
        float distance = sqrt(enemy_x * enemy_x + enemy_y * enemy_y);
        
        // 根据距离确定舵机移动速度
        int move_speed = 1000;  // 默认速度
        if (distance < RADAR_MIN_RANGE) {
            move_speed = 1500;  // 目标太近，快速响应
        } else if (distance > RADAR_MAX_RANGE * 0.7) {
            move_speed = 1000;  // 目标较远，慢速移动
        }
        
        // 更新雷达舵机位置
        radar_servo_position = target_position;
        
        // 设置水平舵机到雷达指示位置
        Servo_SetPosition(1, target_position, move_speed);
        
        // 设置垂直舵机到中间位置
        Servo_SetPosition(2, servo_y, move_speed);
        
        // 检查预瞄准是否完成
        if (radar_aiming_counter <= 0) {
            radar_aiming = 0;
            system_mode = MODE_CAMERA_TRACKING;  // 切换到摄像头追踪模式
            
            // 初始化摄像头追踪状态
            velocity_x = 0.0f;
            velocity_y = 0.0f;
            fine_servo_x.base = target_position;
            fine_servo_x.fine = 0;
            servo_x = target_position;  // 保持当前雷达位置
        }
        
        // 重置锁定计数
        lock_count = 0;
        return;  // 预瞄准期间跳过后续处理
    }
    
    // 非预瞄准状态下的处理（雷达扫描模式）
    if (system_mode == MODE_RADAR_SCANNING) {
        // 添加雷达扫描模式的具体行为（如果需要）
        // 例如：执行扫描模式，寻找目标
    }
}

void control_camera(int obj_x, int obj_y) {
    // 如果处于雷达扫描模式，不进行摄像头追踪
    if (system_mode != MODE_CAMERA_TRACKING) {
        return;
    }
    
    // 计算物体移动方向
    int delta_x = obj_x - last_obj_x;
    int delta_y = obj_y - last_obj_y;
    
    // 更新位置历史
    last_obj_x = obj_x;
    last_obj_y = obj_y;
    
    // 计算目标速度（低通滤波平滑）
    const float ALPHA = 0.15f;
    velocity_x = ALPHA * delta_x + (1 - ALPHA) * velocity_x;
    velocity_y = ALPHA * delta_y + (1 - ALPHA) * velocity_y;
    
    // 应用运动预测
    int predicted_x = obj_x;
    int predicted_y = obj_y;
    
    if (abs(delta_x) > MIN_MOVE_FOR_PREDICTION || abs(delta_y) > MIN_MOVE_FOR_PREDICTION) {
        predicted_x = obj_x + (int)(PREDICTION_FACTOR * velocity_x);
        predicted_y = obj_y + (int)(PREDICTION_FACTOR * velocity_y);
    }
    
    // 计算图像坐标误差
    int error_x = CAMERA_CENTER_X - predicted_x;
    int error_y = CAMERA_CENTER_Y - predicted_y;
    
    // 超小步进控制参数
    const float FINE_STEP_SIZE = 0.2f;
    float fine_step_x = 0;
    float fine_step_y = 0;
    
    // 移动间隔控制
    move_counter = (move_counter + 1) % 1;
    bool allow_move = (move_counter == 0);
    
    // 锁定阈值
    int LOCK_THRESHOLD_X = 40;
    int LOCK_THRESHOLD_Y = 12;


if (y1<(257-(y3-y1)/50-2)&&y3>(257+(y3-y1)/50+2)&&x1<(371-(x3-x1)/50-2)&&x3>(371+(x3-x1)/50+2)) {
        Lock_count++;
        temp = 0;
    } else {
        temp++;
        if (temp >= 50) {
        if(Lock_count<205)
          Lock_count = 0;
        }
    }

    if (Lock_count >= 205&&Lock_count < 300) {
            //jiaojiaojiao
            buzzer_mark = 1;
            Lock_count++;
    }
    if(Lock_count>=300){
        Lock_count=0;
        buzzer_mark=0;
    }


    // 检查锁定条件
    if (abs(error_x) <= LOCK_THRESHOLD_X && abs(error_y) <= LOCK_THRESHOLD_Y) {
        lock_count++;
        
        if (lock_count >= 2) {
            // 锁定状态
            consecutive_steps = 0;
            velocity_x = 0.0f;
            velocity_y = 0.0f;
            return;
        }
    } else {
        lock_count = 0;
    }
    
    

    // 水平方向微步控制
    if (allow_move && abs(error_x) > LOCK_THRESHOLD_X) {
        float dynamic_step1 = 8*FINE_STEP_SIZE;
        if (abs(error_x) > 150) {
            dynamic_step1 *= 1.5f;
        }
        if (abs(error_x) < 40) {
            dynamic_step1 *= 0.4f;
        }
        fine_step_x = (error_x > 0) ? -dynamic_step1 : dynamic_step1;
    }
    
    // 垂直方向微步控制
    if (allow_move && abs(error_y) > LOCK_THRESHOLD_Y) {
        float dynamic_step2 = 1.2*FINE_STEP_SIZE;
        if (abs(error_y) > 200) {
            dynamic_step2 *= 1.5f;
        }
        fine_step_y = (error_y > 0) ? dynamic_step2 : -dynamic_step2;
    }
    
    // 应用精细控制
    if (fine_step_x != 0) {
        fine_servo_x.fine += fine_step_x;
        
        if (fabs(fine_servo_x.fine) >= 1.0f) {
            int actual_step = (int)truncf(fine_servo_x.fine);
            fine_servo_x.base = constrain(fine_servo_x.base + actual_step, 1000, 3000);
            fine_servo_x.fine -= actual_step;
            consecutive_steps++;
            servo_x = fine_servo_x.base;
        }
    }
    
    if (fine_step_y != 0) {
        fine_servo_y.fine += fine_step_y;
        
        if (fabs(fine_servo_y.fine) >= 1.0f) {
            int actual_step = (int)truncf(fine_servo_y.fine);
            fine_servo_y.base = constrain(fine_servo_y.base + actual_step, 1950, 2350);
            fine_servo_y.fine -= actual_step;
            consecutive_steps++;
            servo_y = fine_servo_y.base;
        }
    }
    
    // 发送指令
    static int last_servo_x = 1900;
    static int last_servo_y = 2200;
    
    if (servo_x != last_servo_x || servo_y != last_servo_y) {
        if (servo_x != last_servo_x) {
            int speed = (abs(velocity_x) > 5) ? 300 : 500;
            Servo_SetPosition(1, servo_x, speed);
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
