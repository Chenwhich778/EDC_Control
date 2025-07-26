#include "servo_protocol.h"
#include <string.h>

char coord[BUF_SIZE] = {0};
volatile uint8_t coord_updated = 0;
// ����ָ����
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

void Servo_SetPosition(uint8_t id, uint16_t position, uint16_t speed) {
    uint8_t packet[13];  // Packet buffer (13 bytes)
    uint8_t params[6] = {
        (uint8_t)(position & 0xFF),  // Position low byte
        (uint8_t)(position >> 8),    // Position high byte
        0x00,                       // Time low byte (unused)
        0x00,                       // Time high byte (unused)
        (uint8_t)(speed & 0xFF),     // Speed low byte
        (uint8_t)(speed >> 8)        // Speed high byte
    };

   // 1. Build the packet
    BuildPacket(id, 0x03, 0x2A, params, 6, packet);

    // 2. Send each byte via fputc (更高效！)
   uart1_send(packet, 13);       // 发送13字节数据
   uart1_send((uint8_t*)"\n", 1); // 追加换行符
}


