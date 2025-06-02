#include "servo_protocol.h"
#include <string.h>

static UART_HandleTypeDef *servo_huart;
extern UART_HandleTypeDef huart3;

// 舵机通信相关变量
static uint8_t rx_buf[BUF_SIZE];
static volatile uint8_t rx_flag = 0;

// USART3接收相关变量
char coord[BUF_SIZE] = {0};
volatile uint16_t rx_index = 0;
volatile uint8_t rx_complete = 0;
volatile uint8_t coord_updated = 0;  // 新增：用于标记coord已更新

// 初始化USART6和舵机通信
void Servo_Init(UART_HandleTypeDef *huart) {
    servo_huart = huart;
    HAL_UART_Receive_IT(servo_huart, rx_buf, BUF_SIZE);
    
    // 初始化USART3接收
    HAL_UART_Receive_IT(&huart3, (uint8_t*)&coord[rx_index], 1);
}

// 构建指令包
static void BuildPacket(uint8_t id, uint8_t cmd, uint8_t addr, uint8_t *params, uint8_t param_len, uint8_t *packet) {
    uint8_t checksum = 0;
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = param_len + 3;
    packet[4] = cmd;
    packet[5] = addr;
    memcpy(&packet[6], params, param_len);
    for(int i=2; i<6+param_len; i++) checksum += packet[i];
    packet[6+param_len] = ~checksum;
}

// 设置舵机位置和速度
void Servo_SetPosition(uint8_t id, uint16_t position, uint16_t speed) {
    uint8_t packet[13]; // 修正为13字节
    uint8_t params[6] = {
        (uint8_t)(position & 0xFF),      // 位置低字节
        (uint8_t)(position >> 8),        // 位置高字节 
        0x00,                           // 时间低字节
        0x00,                           // 时间高字节
        (uint8_t)(speed & 0xFF),         // 速度低字节
        (uint8_t)(speed >> 8)            // 速度高字节
    };
    
    BuildPacket(id, 0x03, 0x2A, params, 6, packet);
    HAL_UART_Transmit(servo_huart, packet, 13, HAL_MAX_DELAY); // 发送13字节
}

//// 同时控制两个舵机的位置和速度
//void Servo_SetREDLaser(uint8_t id1, uint16_t pos1, uint16_t speed1, 
//                            uint8_t id2, uint16_t pos2, uint16_t speed2) {
//    const uint8_t param_len_per_servo = 4; // 每个舵机的参数长度(位置2字节+速度2字节)
//    const uint8_t total_param_len = 1 + 2 + 2 * (1 + param_len_per_servo); // 总参数长度
//    uint8_t packet[2 + 1 + 1 + 1 + total_param_len + 1]; // 完整包长度
//    
//    // 构建SYNC WRITE指令包
//    uint8_t index = 0;
//    packet[index++] = 0xFF; // 帧头
//    packet[index++] = 0xFF; // 帧头
//    packet[index++] = 0xFE; // 广播ID
//    packet[index++] = total_param_len + 2; // 长度
//    packet[index++] = 0x83; // SYNC WRITE指令
//    packet[index++] = 0x2A; // 目标位置寄存器地址
//    packet[index++] = param_len_per_servo; // 每个舵机的参数长度
//    
//    // 第一个舵机参数
//    packet[index++] = id1;
//    packet[index++] = (uint8_t)(pos1 & 0xFF); // 位置低位
//    packet[index++] = (uint8_t)(pos1 >> 8);   // 位置高位
//    packet[index++] = 0x00; // 时间低位(设为0)
//    packet[index++] = 0x00; // 时间高位(设为0)
//    packet[index++] = (uint8_t)(speed1 & 0xFF); // 速度低位
//    packet[index++] = (uint8_t)(speed1 >> 8);   // 速度高位
//    
//    // 第二个舵机参数
//    packet[index++] = id2;
//    packet[index++] = (uint8_t)(pos2 & 0xFF); // 位置低位
//    packet[index++] = (uint8_t)(pos2 >> 8);   // 位置高位
//    packet[index++] = 0x00; // 时间低位(设为0)
//    packet[index++] = 0x00; // 时间高位(设为0)
//    packet[index++] = (uint8_t)(speed2 & 0xFF); // 速度低位
//    packet[index++] = (uint8_t)(speed2 >> 8);   // 速度高位
//    
//    // 计算校验和
//    uint8_t checksum = 0;
//    for (uint8_t i = 2; i < index; i++) checksum += packet[i];
//    packet[index++] = ~checksum;
//    
//    // 发送数据包
//    HAL_UART_Transmit(servo_huart, packet, index, HAL_MAX_DELAY);
//}







// 读取舵机位置
uint16_t Servo_ReadPosition(uint8_t id) {
    uint8_t packet[8];
    uint8_t params[2] = {GOAL_POSITION_ADDR, 0x02};
    BuildPacket(id, READ_DATA, GOAL_POSITION_ADDR, params, 2, packet);
    HAL_UART_Transmit(servo_huart, packet, 8, HAL_MAX_DELAY);
    
    uint32_t tickstart = HAL_GetTick();
    while(!rx_flag && (HAL_GetTick() - tickstart < SERVO_TIMEOUT));
    
    if(rx_flag && rx_buf[4] == 0x00) {
        return (rx_buf[6] << 8) | rx_buf[5];
    }
    return 0xFFFF;
}

// 获取最新coord数据
void Servo_GetCoord(char *dest, uint16_t max_len) {
//    __disable_irq();  // 防止读取过程中被中断修改
    strncpy(dest, coord, max_len - 1);
    dest[max_len - 1] = '\0';
    coord_updated = 0;  // 清除更新标志
//    __enable_irq();
}

// USART接收中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == servo_huart) {
        // 舵机数据接收处理
        rx_flag = 1;
        HAL_UART_Receive_IT(servo_huart, rx_buf, BUF_SIZE);
    }
    else if (huart->Instance == USART3) {
        if (coord[rx_index] == '\n' || rx_index >= BUF_SIZE - 1) {
            coord[rx_index] = '\0';  // 终止字符串
            rx_complete = 1;
            coord_updated = 1;       // 标记数据可用
            
            // 仅重置索引，不立即清空缓冲区
            rx_index = 0;
            
            // 立即重启接收（新数据将覆盖旧内容）
            HAL_UART_Receive_IT(&huart3, (uint8_t*)&coord[rx_index], 1);
        }
    
		else {
            rx_index++;  // 递增索引
            HAL_UART_Receive_IT(&huart3, (uint8_t*)&coord[rx_index], 1);
        }
    }
}

