#ifndef __SERVO_PROTOCOL_H__
#define __SERVO_PROTOCOL_H__
#include "stdio.h"
#include "sys.h"
#include "system.h"

#define BUF_SIZE 64
#define SERVO_TIMEOUT 100
#define GOAL_POSITION_ADDR 0x2A
#define READ_DATA 0x02
#define prime_x1 2000
#define prime_y1 2000
#define max_servo 4095

extern char coord[BUF_SIZE];
extern volatile uint8_t coord_updated;

void Servo_SetPosition(uint8_t id, uint16_t position, uint16_t speed);

#endif

