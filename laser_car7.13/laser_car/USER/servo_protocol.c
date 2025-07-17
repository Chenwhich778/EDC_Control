#include "servo_protocol.h"
#include <string.h>

char coord[BUF_SIZE] = {0};
volatile uint8_t coord_updated = 0;
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
    
    // 循环发送整个数据包
    for (uint8_t i = 0; i < 13; i++) {
        usart6_send(packet[i]);
    }
}


