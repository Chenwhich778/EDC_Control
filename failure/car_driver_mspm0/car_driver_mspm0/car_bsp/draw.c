// #include "car_bsp.h"

// CircleDrawer circle_drawer = {0};

// // 初始化画圆参数
// void start_circle_drawing(uint16_t center_x, uint16_t center_y, uint16_t radius,
//                          uint16_t speed, uint16_t steps) {
//     circle_drawer.center_x = center_x;
//     circle_drawer.center_y = center_y;
//     circle_drawer.radius = radius;
//     circle_drawer.speed = speed;
//     circle_drawer.steps = (steps < 36) ? 36 : steps; // 至少36步保证圆形
//     circle_drawer.current_step = 0;
//     circle_drawer.is_active = true;
// }

// // 执行一步画圆操作（放在主循环中调用）
// void update_circle_drawing() {
//     if (!circle_drawer.is_active) return;

//     // 计算当前角度（弧度）
//     float angle = 2 * M_PI * circle_drawer.current_step / circle_drawer.steps;
    
//     // 计算圆上点的坐标
//     int16_t dx = (int16_t)(circle_drawer.radius * cosf(angle));
//     int16_t dy = (int16_t)(circle_drawer.radius * sinf(angle));
    
//     // 计算舵机目标位置
//     uint16_t target_x = circle_drawer.center_x + dx;
//     uint16_t target_y = circle_drawer.center_y + dy;
    
//     // 限制舵机位置在0-4095范围内
//     target_x = (target_x > 4095) ? 4095 : target_x;
//     target_y = (target_y > 4095) ? 4095 : target_y;
//     target_x = (target_x < 0) ? 0 : target_x;
//     target_y = (target_y < 0) ? 0 : target_y;
    
//     // 控制舵机
//     Servo_SetPosition(1, target_x, circle_drawer.speed); // 水平舵机
//     Servo_SetPosition(2, target_y, circle_drawer.speed); // 垂直舵机
    
//     // 更新到下一步
//     circle_drawer.current_step++;
//     if (circle_drawer.current_step >= circle_drawer.steps) {
//         circle_drawer.current_step = 0; // 循环画圆
//     }
    
//     // 可选：添加小延时保证舵机响应
//     // delay_ms(1); 
// }

// // 停止画圆
// void stop_circle_drawing() {
//     circle_drawer.is_active = false;
//     // 回到中心位置
//     Servo_SetPosition(1, circle_drawer.center_x, circle_drawer.speed);
//     Servo_SetPosition(2, circle_drawer.center_y, circle_drawer.speed);
// }










